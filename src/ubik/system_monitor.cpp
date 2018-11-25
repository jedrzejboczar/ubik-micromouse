#include "system_monitor.h"

#include "movement/regulator.h"
#include "ubik/logging/logging.h"


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

void lock_button() {
    // this should always succeed as we wait indefinitelly
    bool taken = xSemaphoreTake(button_mutex, portMAX_DELAY) == pdPASS;
    configASSERT(taken);
}

void unlock_button() {
    // this could only fail if we didn't take the semaphore earlier
    bool could_give = xSemaphoreGive(button_mutex) == pdPASS;
    configASSERT(could_give);
}

bool wait_for_button_press(uint32_t max_wait_time_ms) {
    bool button_pressed = false;
    auto start_time = xTaskGetTickCount();
    while (!button_pressed && xTaskGetTickCount() - start_time < pdMS_TO_TICKS(max_wait_time_ms)) {
        button_pressed = sampled_check_button();
    }
    return button_pressed;
}

float select_with_wheels(float min_value, float max_value,
        float value_per_left_wheel_turn, float value_per_right_wheel_turn,
        float initial_value, const char *print_prompt)
{
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
        float ticks_per_turn = localization::MAX_ENCODER_READING * constants::GEAR_RATIO;
        value = initial_value
            + static_cast<float>(left) / ticks_per_turn * value_per_left_wheel_turn
            + static_cast<float>(right) / ticks_per_turn * value_per_right_wheel_turn;

        // saturate
        value = std::max(min_value, value);
        value = std::min(value, max_value);

        // print current value
        if (print_prompt != nullptr)
            logging::printf(100, "%s %.3f\n", print_prompt, static_cast<double>(value));
    }

    // re-enable regulation
    regulation_state = last_regulation_state;

    if (print_prompt != nullptr)
        logging::printf(100, "Selected: %.3f\n", static_cast<double>(value));

    return value;
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
                logging::printf(80, "[# CRITICAL #] Voltage below critical level: %d < %d [mV]\n",
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
                    logging::printf(80, "[# WARNING #] Voltage below warning level: %d < %d [mV]\n",
                        static_cast<int>(filtered_voltage * 1e3f), static_cast<int>(WARING_VOLTAGE * 1e3f));
                    last_warning = xTaskGetTickCount();
                }
            }
        }

        // process the regulation state changes
        update_regulation();

        // print current/voltage if requested
        if constexpr (PRINT_BATTERY_MEASUREMENTS_EVERY > 0) {
            static uint32_t counter = 0;
            if (counter++ >= PRINT_BATTERY_MEASUREMENTS_EVERY - 1) {
                counter = 0;
                logging::printf(80, "Voltage level: %.3f V   Current level: %.3f A\n",
                        static_cast<double>(filtered_voltage), static_cast<double>(current));
            }
        }

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
        logging::printf(50, "Regulation %s\n", current_regulation_state ? "ON" : "OFF");
        movement::regulator::set_enabled(current_regulation_state);
        last_state = current_regulation_state;
    }
}

} // namespace system_monitor
