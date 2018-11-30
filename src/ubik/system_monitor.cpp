#include "system_monitor.h"

#include "ubik/movement/regulator.h"
#include "ubik/logging/logging.h"

#include "ubik/localization/odometry.h"

namespace system_monitor {



// system monitor task loop frequency
static constexpr int LOOP_PERIOD_MS = 1000 / LOOP_FREQ_HZ;
static constexpr int VOLTAGE_WARINGS_PERIOD_MS = 1000 / VOLTAGE_WARINGS_FREQ_HZ;

static_assert(LOOP_PERIOD_MS > N_BUTTON_MEASUREMENTS * TIME_BETWEEN_MEASUREMENTS_MS + 2 + 5,
        "Loop period must be > total button sampling time + time for ADC + error margin for calculations");

// ADC used, TODO: when we need line sensors this must be rewritten/reconfigured somehow
extern "C" ADC_HandleTypeDef hadc2; // 3 channels: 2 for voltage and current, 1 for line sensors
ADC_HandleTypeDef &hadc = hadc2;

// mutex to allow for usage of the button from other tasks
static SemaphoreHandle_t button_mutex = nullptr;

// keep track of regulation state, write/read to an int _should_ be atomic
static bool regulation_state = false;

static bool system_monitor_task_ready = false;

/******************************************************************************/

static bool check_button();
static bool sampled_check_button();
static void update_regulation_state(bool force=false);
static float adc2voltage(float adc_reading);
static float adc2current(float adc_reading);
static std::pair<float, float> measure_current_and_voltage();

void initialise() {
    button_mutex = xSemaphoreCreateMutex();
    configASSERT(button_mutex != nullptr);
}

bool get_regulation_state() {
    return regulation_state;
}

void lock_button() {
    // prevent problems when task locks button faster than system monitor
    // TODO: for this we should use event groups
    while (!system_monitor_task_ready)
        vTaskDelay(1);
    // this should always succeed as we wait indefinitelly
    bool taken = xSemaphoreTake(button_mutex, portMAX_DELAY) == pdPASS;
    configASSERT(taken);
}

void unlock_button() {
    // this could only fail if we didn't take the semaphore earlier
    bool could_give = xSemaphoreGive(button_mutex) == pdPASS;
    configASSERT(could_give);
}

bool is_button_locked() {
    return uxSemaphoreGetCount(button_mutex) == 0;
}

bool wait_for_button_press(uint32_t max_wait_time_ticks) {
    bool button_pressed = false;
    auto start_time = xTaskGetTickCount();
    while (!button_pressed && xTaskGetTickCount() - start_time < max_wait_time_ticks) {
        button_pressed = sampled_check_button();
    }
    return button_pressed;
}



static float select_with_wheels_generic(float initial_value, std::pair<float, float> values_range,
        float change_per_left_wheel_turn, const char *print_prompt, bool is_int)
{
    auto to_int_value = [] (float value) { return static_cast<int>(std::round(value)); };

    float last_value = std::numeric_limits<float>::max();
    float value = initial_value;
    int32_t initial_left, initial_right;
    std::tie(initial_left, initial_right) = localization::get_cumulative_encoder_ticks();

    // disable regulation to allow free wheel turning
    bool last_regulation_state = regulation_state;
    regulation_state = false;

    // continue until the button is pressed
    while (!sampled_check_button()) {

        // get the current ticks
        int32_t left, right;
        std::tie(left, right) = localization::get_cumulative_encoder_ticks();

        // correct according to initial positions
        left = initial_left - left;
        right = initial_right - right;

        // calculate current value
        const float ticks_per_turn = localization::MAX_ENCODER_READING * constants::GEAR_RATIO;
        const float change_per_right_wheel_turn = /* is_int ? 1 : */
            change_per_left_wheel_turn/SELECTION_RIGHT_WHEEL_RESOLUTION_RATIO;
        value = initial_value
            + left / ticks_per_turn * change_per_left_wheel_turn
            + right / ticks_per_turn * change_per_right_wheel_turn;

        // saturate
        value = std::max(values_range.first, value);
        value = std::min(value, values_range.second);

        // print current value
        if (print_prompt != nullptr) {
            float delta = (values_range.second - values_range.first) * MIN_PRINTED_DIFFERENCE_RATIO;
            bool changed_enough;

            if (is_int) {
                changed_enough = std::abs( to_int_value(value) - to_int_value(last_value) ) > 0;
            } else {
                changed_enough = std::abs(value - last_value) > delta;
            }

            if (changed_enough) {
                const char *suffix;
                if (is_int) {
                    suffix = to_int_value(value) == to_int_value(values_range.second) ? " (max)" :
                        to_int_value(value) == to_int_value(values_range.first) ? " (min)" : "";
                } else {
                    suffix = value > values_range.second - delta ? " (max)" :
                        value < values_range.first + delta ? " (min)" : "";
                }

                if (is_int)
                    logging::printf(100, "%s %d%s\n", print_prompt, to_int_value(value), suffix);
                else
                    logging::printf(100, "%s %.3f%s\n", print_prompt, static_cast<double>(value), suffix);

                last_value = value;
            }

        }
    }

    // re-enable regulation
    regulation_state = last_regulation_state;

    if (print_prompt != nullptr) {
        if (is_int)
            logging::printf(100, "Selected: %d\n", to_int_value(value));
        else
            logging::printf(100, "Selected: %.3f\n", static_cast<double>(value));
    }

    if (is_int)
        value = to_int_value(value);
    return value;
}


float select_with_wheels(float initial_value, std::pair<float, float> values_range,
        float change_per_left_wheel_turn, const char *print_prompt)
{
    return select_with_wheels_generic(initial_value, values_range,
        change_per_left_wheel_turn, print_prompt, false);
}

int select_with_wheels_int(int initial_value, std::pair<int, int> values_range,
        int increments_per_left_wheel_turn, const char *print_prompt)
{
    return select_with_wheels_generic(initial_value, values_range,
        increments_per_left_wheel_turn, print_prompt, true);
}




/******************************************************************************/

void system_monitor_task(void *) {
    // current regulation state
    bool regulation_on_button = REGULATION_START_ON;
    bool regulation_on_voltage = false;
    // voltage=false always disables, when it's true, then buttons enables/disables
    auto regulation_enabled = [&regulation_on_button, &regulation_on_voltage] () {
        return regulation_on_voltage && regulation_on_button;
    };

    // Update task internal regulation state, process the real changes.
    // It got pretty complicated, but it is needed to allow for enabling/disabling
    // regulation outside the task, e.g. in select_with_wheels().
    auto update_regulation = [&regulation_enabled] () {
        // force first change by opposite initial value
        static bool last_enabled = !regulation_enabled();
        if (regulation_enabled() != last_enabled) {
            regulation_state = regulation_enabled();
            last_enabled = regulation_enabled();
        }
        update_regulation_state();
    };
    update_regulation_state(true);

    // calibrate ADC (this function waits actually for end of calibartion, even if its name is _Start)
    auto calibration_result = HAL_ADCEx_Calibration_Start(&hadc);
    configASSERT(calibration_result == HAL_OK);

    // get initial values (for filter)
    float current, voltage;
    std::tie(current, voltage) = measure_current_and_voltage();
    float last_voltage_raw = voltage;
    float filtered_voltage = voltage;

    // calculates next filter output
    auto iir_filter_next = [&last_voltage_raw, &filtered_voltage] (float current_voltage_x) {
        filtered_voltage =
            VOLTAGE_IIR_B[0] * current_voltage_x +
            VOLTAGE_IIR_B[1] * last_voltage_raw -
            VOLTAGE_IIR_A[0] * filtered_voltage;
        last_voltage_raw = current_voltage_x;
    };

    // timings
    auto last_start = xTaskGetTickCount();
    auto last_warning = xTaskGetTickCount();
    auto last_critical = xTaskGetTickCount();

    while (1) {
        // check button if noone uses it at the moment, do not block
        if (xSemaphoreTake(button_mutex, 0) == pdPASS) {
            // toggle regulation state on button press
            if (sampled_check_button())
                regulation_on_button = !regulation_on_button;
            unlock_button();
        }

        // measure and calculate next filter output
        std::tie(current, voltage) = measure_current_and_voltage();
        iir_filter_next(voltage);

        // check the voltage
        if (filtered_voltage < CRITICAL_VOLTAGE) {
            if (xTaskGetTickCount() - last_critical > pdMS_TO_TICKS(VOLTAGE_WARINGS_PERIOD_MS)) {
                logging::printf(80, "[# CRITICAL #] [sys] Voltage below critical level: %d < %d [mV]\n",
                        static_cast<int>(filtered_voltage * 1e3f), static_cast<int>(CRITICAL_VOLTAGE * 1e3f));
                last_critical = xTaskGetTickCount();
            }
            if (IS_BATTERY_SUPPLY)
                regulation_on_voltage = false;
        } else {
            // re-enable motors
            regulation_on_voltage = true;
            if (filtered_voltage < WARING_VOLTAGE) {
                if (xTaskGetTickCount() - last_warning > pdMS_TO_TICKS(VOLTAGE_WARINGS_PERIOD_MS)) {
                    logging::printf(80, "[# WARNING #] [sys] Voltage below warning level: %d < %d [mV]\n",
                        static_cast<int>(filtered_voltage * 1e3f), static_cast<int>(WARING_VOLTAGE * 1e3f));
                    last_warning = xTaskGetTickCount();
                }
            }
        }

        // turn on/off the warning LED
        if (filtered_voltage < WARING_VOLTAGE)  // warning OR critical
            spi::gpio::update_pins(spi::gpio::LED_RED, 0);
        else
            spi::gpio::update_pins(0, spi::gpio::LED_RED);

        // process the regulation state changes
        update_regulation();

        // print current/voltage if requested
        if constexpr (PRINT_BATTERY_MEASUREMENTS_EVERY > 0) {
            static uint32_t counter = PRINT_BATTERY_MEASUREMENTS_EVERY - 1;
            if (counter++ >= PRINT_BATTERY_MEASUREMENTS_EVERY - 1) {
                counter = 0;
                logging::printf(80, "[sys] Voltage level: %.3f V   Current level: %.3f A\n",
                        static_cast<double>(filtered_voltage), static_cast<double>(current));
            }
        }

        system_monitor_task_ready = true;
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

/******************************************************************************/

static bool sampled_check_button_generic(int dt_ms, int n_samples, int min_n_samples) {
    // to avoid subsequent reads after registered one
    static int last_button_press = std::numeric_limits<int>::min();
    if (xTaskGetTickCount() - last_button_press < pdMS_TO_TICKS(MIN_BUTTON_DELAY_MS))
        return false;

    // sample the button for <100 ms
    int button_on_count = 0;
    for (int i = 0; i < n_samples; i++) {
        if (check_button())
            button_on_count++;
        vTaskDelay(pdMS_TO_TICKS(dt_ms));
    }

    // register new press time if pressed
    bool pressed = button_on_count > min_n_samples;
    if (pressed)
        last_button_press = xTaskGetTickCount();

    return pressed;
}

static bool sampled_check_button() {
    return sampled_check_button_generic(
            TIME_BETWEEN_MEASUREMENTS_MS, N_BUTTON_MEASUREMENTS, MIN_BUTTON_ON_COUNT);
}

static bool check_button() {
    return HAL_GPIO_ReadPin(SW_Start_GPIO_Port, SW_Start_Pin) == GPIO_PIN_SET;
}

static float adc2voltage(float adc_reading) {
    return adc_reading / MAX_ADC_READING * VCC * (1 / VOLTAGE_RESISTOR_DIVIDER);
}

static float adc2current(float adc_reading) {
    float voltage = adc_reading / MAX_ADC_READING * VCC * CURRENT_READING_GAIN;
    return voltage / CURRENT_MEASUREMENT_RESISTOR;
}

static std::pair<float, float> measure_current_and_voltage() {
    // conversion time is about 28us, so we'll just wait for now, we have time
    HAL_ADC_Start(&hadc);
    vTaskDelay(pdMS_TO_TICKS(1)); // wait MUCH too long for conversion
    int current = HAL_ADC_GetValue(&hadc); // ignore current measurements

    HAL_ADC_Start(&hadc);
    vTaskDelay(pdMS_TO_TICKS(1)); // wait MUCH too long for conversion
    int voltage = HAL_ADC_GetValue(&hadc);

    return std::make_pair(adc2current(current), adc2voltage(voltage));
}

static void update_regulation_state(bool force) {
    static bool last_state = regulation_state;
    // atomic read
    bool current_regulation_state = regulation_state;
    if (current_regulation_state != last_state || force) {
        logging::printf(50, "[sys] Regulation %s\n", current_regulation_state ? "ON" : "OFF");
        movement::regulator::set_enabled(current_regulation_state);
        last_state = current_regulation_state;
    }
}

} // namespace system_monitor
