#include "system_monitor.h"

#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "movement/regulator.h"
#include "ubik/logging/logging.h"


// ADC used, TODO: when we need line sensors this must be rewritten/reconfigured somehow
extern "C" ADC_HandleTypeDef hadc2; // 3 channels: 2 for voltage and current, 1 for line sensors
ADC_HandleTypeDef &hadc = hadc2;

// system monitor task loop frequency
static constexpr int LOOP_PERIOD_MS = 1000 / LOOP_FREQ_HZ;
static constexpr int VOLTAGE_WARINGS_PERIOD_MS = 1000 / VOLTAGE_WARINGS_FREQ_HZ;

static_assert(LOOP_PERIOD_MS > N_BUTTON_MEASUREMENTS * TIME_BETWEEN_MEASUREMENTS_MS + 2 + 5,
        "Loop period must be > total button sampling time + time for ADC + error margin for calculations");


static bool check_button();
static float adc2voltage(float adc_reading);


void system_monitor_task(void *) {
    // current regulation state
    bool regulation_on_button = REGULATION_START_ON;
    bool regulation_on_voltage = false;
    // decides if regulation should be enabled or not
    auto set_regulation_state = [&regulation_on_button, &regulation_on_voltage] () {
        static bool enabled = false;
        // voltage=false always disables, when it's true, then buttons enables/disables
        bool new_enabled = regulation_on_voltage && regulation_on_button;
        if (new_enabled != enabled) {
            enabled = new_enabled;
            logging::printf(50, "Regulation %s\n", enabled ? "ON" : "OFF");
            movement::regulator::set_enabled(enabled);
        }
    };

    // calibrate ADC (this function waits actually for end of calibartion, even if its name is _Start)
    auto calibration_result = HAL_ADCEx_Calibration_Start(&hadc);
    configASSERT(calibration_result == HAL_OK);

    // get initial voltage value (first ignore current reading, which is on 1st ADC channel)
    HAL_ADC_Start(&hadc);
    vTaskDelay(pdMS_TO_TICKS(1)); // wait MUCH too long for conversion
    // get initial voltage
    float last_voltage_raw, last_voltage; // x=measured, y=filtered
    HAL_ADC_Start(&hadc);
    vTaskDelay(pdMS_TO_TICKS(1)); // wait MUCH too long for conversion
    last_voltage_raw = last_voltage = adc2voltage(HAL_ADC_GetValue(&hadc));
    // calculates next filter output
    auto iir_filter_next = [&last_voltage_raw, &last_voltage] (float current_voltage_x) {
        last_voltage =
            VOLTAGE_IIR_B[0] * current_voltage_x +
            VOLTAGE_IIR_B[1] * last_voltage_raw -
            VOLTAGE_IIR_A[0] * last_voltage;
        last_voltage_raw = current_voltage_x;
    };

    // timings
    auto last_start = xTaskGetTickCount();
    int last_button_press = std::numeric_limits<int>::min();
    auto last_warning = xTaskGetTickCount();
    auto last_critical = xTaskGetTickCount();

    while (1) {
        // when the button has not been used for at least 1 second, to avoid double-press
        if (xTaskGetTickCount() - last_button_press > pdMS_TO_TICKS(MIN_BUTTON_DELAY_MS)) {
            // sample the button for <100 ms
            int button_on_count = 0;
            for (int i = 0; i < N_BUTTON_MEASUREMENTS; i++) {
                if (check_button())
                    button_on_count++;
                vTaskDelay(pdMS_TO_TICKS(TIME_BETWEEN_MEASUREMENTS_MS));
            }

            // register button press when there were 70-100 % positives
            if (button_on_count > MIN_BUTTON_ON_COUNT) {
                // toggle regulation state on button press
                regulation_on_button = !regulation_on_button;
                // reset button timer
                last_button_press = xTaskGetTickCount();
            }
        }

        // conversion time is about 28us, so we'll just wait for now, we have time
        HAL_ADC_Start(&hadc);
        vTaskDelay(pdMS_TO_TICKS(1)); // wait MUCH too long for conversion
        HAL_ADC_GetValue(&hadc); // ignore current measurements
        HAL_ADC_Start(&hadc);
        vTaskDelay(pdMS_TO_TICKS(1)); // wait MUCH too long for conversion
        int voltage = HAL_ADC_GetValue(&hadc);

        // calculate next filter output
        iir_filter_next(adc2voltage(voltage));

        // check the voltage
        if (last_voltage < CRITICAL_VOLTAGE) {
            if (xTaskGetTickCount() - last_critical > pdMS_TO_TICKS(VOLTAGE_WARINGS_PERIOD_MS)) {
                logging::printf(80, "[# CRITICAL #] Voltage below critical level: %d < %d [mV]\n",
                        static_cast<int>(last_voltage * 1e3f), static_cast<int>(CRITICAL_VOLTAGE * 1e3f));
                last_critical = xTaskGetTickCount();
            }
            if (IS_BATTERY_SUPPLY)
                regulation_on_voltage = false;
        } else {
            // re-enable motors
            regulation_on_voltage = true;
            if (last_voltage < WARING_VOLTAGE) {
                if (xTaskGetTickCount() - last_warning > pdMS_TO_TICKS(VOLTAGE_WARINGS_PERIOD_MS)) {
                    logging::printf(80, "[# WARNING #] Voltage below warning level: %d < %d [mV]\n",
                        static_cast<int>(last_voltage * 1e3f), static_cast<int>(WARING_VOLTAGE * 1e3f));
                    last_warning = xTaskGetTickCount();
                }
            }
        }

        // change the current regulation state if needed
        set_regulation_state();

        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}


bool check_button() {
    return HAL_GPIO_ReadPin(SW_Start_GPIO_Port, SW_Start_Pin) == GPIO_PIN_SET;
}

float adc2voltage(float adc_reading) {
    return adc_reading / MAX_ADC_READING * VCC * (1 / VOLTAGE_RESISTOR_DIVIDER);
}



