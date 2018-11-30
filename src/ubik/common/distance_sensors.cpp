#include "distance_sensors.h"

#include <cmath>

namespace distance_sensors {

/*
 * ADC configuration:
 * - scan mode on
 * - continuous mode off
 * - discontinuous mode off
 * - ofc right aligned
 *
 * ADC1, fclk = 9MHz
 * single conversion: Ts = (12.5 + 28.5) * 1/fclk ~= 4.556 us
 * 6 conversions: T6 = 27.333 us
 * DMA: periph(no-inc)->memory(inc), not-circular, half-word
 *
 * Numeration:
 * - in schematics:
 *                   Front
 *          _______________________
 *  Left   /      o         o      \   Right
 *        /      /           \      \
 *       /      /             \      \
 *      |       DS4         DS3      |
 *      |  o                      o  |
 *      |  |                      |  |
 *      |   DS5               DS2    |
 *      |                            |
 *      | o-- DS6            DS1 --o |
 *      |                            |
 *
 * - sensor to ADC1 channel mapping: DS(N) -> CH(N-1)
 * - in code we use 0-indexing: DS(N) -> DS(N-1)
 * Conversion regular channels order: 0,1,2,3,4,5
 *   (instead of the old_ubik's 0,2,4,1,3,5 that was using discontinuous conversions)
 *
 * Gpio expander used for turning LEDs on/off:
 *    LEDS are connected in different order than in pcb/plots/ubik.pdf TODO: update this pdf
 *    they are actually connected in the order: GPIOEX 0-5 --> DS 4,3,2,1,5,6
 *
 */
extern "C" ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef &ds_hadc = hadc1;
static constexpr uint32_t ADC_SAMPLING_TIME = ADC_SAMPLETIME_28CYCLES5_SMPR1ALLCHANNELS;
static constexpr uint32_t MAX_ADC_WAIT_TIME_MS = 1;

/*
 * Should be configured such that CNT is incremented once each microsecond
 * and the counting mode should be "counting up".
 * The prescaler should be equal to: pre_prescaler_frequency / 10^6 - 1,
 * e.g. for clock of 72MHz, prescaler = 72MHz / 1MHz - 1 = 72 - 1
 */
extern "C" TIM_HandleTypeDef htim2;
TIM_HandleTypeDef &ds_htim = htim2;


static SemaphoreHandle_t adc_mutex = nullptr;
static TaskHandle_t task_to_notify = nullptr;

static int configure_adc_channels(uint8_t sensors);
static bool wait_for_notification();
static bool delay_us(int us);
static void lock();
static void unlock();

void initialise() {
    bool ok = HAL_ADCEx_Calibration_Start(&ds_hadc) == HAL_OK;
    configASSERT(ok);
    adc_mutex = xSemaphoreCreateMutex();
    configASSERT(adc_mutex != nullptr);
}

Readings read(uint8_t gpio_ex_sensors) {
    // mask input argument
    gpio_ex_sensors &= spi::gpio::DISTANCE_SENSORS_ALL();

    // readings buffers
    uint16_t readings_off[6] = {0};
    uint16_t readings_on[6] = {0};

    // obtain the mutex
    lock();

    // set the task for following calls to wait_for_notification
    task_to_notify = xTaskGetCurrentTaskHandle();

    bool readings_ok = false;
    const int n_readings = configure_adc_channels(gpio_ex_sensors);

    if (n_readings > 0) {
        // measure the ambient light levels
        if (HAL_ADC_Start_DMA(&ds_hadc, reinterpret_cast<uint32_t *>(readings_off), n_readings) == HAL_OK) {

            // wait for ADC measurement end
            if (wait_for_notification()) {

                // try to overcome slow SPI problem
                ds_htim.Instance->ARR = 1000;
                ds_htim.Instance->CNT = 0;
                // do not use HAL_TIM_Base_Start to avoid wasting time for function call
                __HAL_TIM_ENABLE(&ds_htim);

                // turn on required LEDs, remember to always turn them off later!
                if (spi::gpio::update_pins(gpio_ex_sensors, 0)) {

                    // update_pins() takes huge amount of time (around 75 microseconds)
                    // writing 3 bytes takes around 51 us, so let's assume that
                    // the time usage of update_pins() is: 12+51+12 us
                    // one byte is transfered in ~17 us, gpio-exp pin set time should be negligible
                    // so, probably, the pin is on after ~12 us (0-24 depending if lock() or unlock() is faster)
                    // calculate the delay time based on our assumptions, CNT should be ~75
                    int32_t delay_time_us = LEDS_TURN_ON_WAIT_TIME_US - (ds_htim.Instance->CNT - (51 + 12));
                    delay_time_us = std::max(int32_t(0), delay_time_us);
                    // do not use HAL_TIM_Base_Stop to avoid wasting time for function call
                    __HAL_TIM_DISABLE(&ds_htim);

                    // & wait until they fully turn on
                    bool delay_ok = delay_us(delay_time_us);

                    // measure the light levels registered
                    if (HAL_ADC_Start_DMA(&ds_hadc, reinterpret_cast<uint32_t *>(readings_on), n_readings) == HAL_OK) {
                        readings_ok = wait_for_notification() && delay_ok;
                    }

                    // remember to turn off all LEDs
                    if (! spi::gpio::update_pins(0, spi::gpio::DISTANCE_SENSORS_ALL()) )  {
                        // this is redundant, but try twice, as it is quite important
                        portYIELD();
                        spi::gpio::update_pins(0, spi::gpio::DISTANCE_SENSORS_ALL());
                    }

                } else {
                    // if the call failed, we haven't disabled the timer, so do it
                    __HAL_TIM_DISABLE(&ds_htim);
                }
            }
        }
    }

    // clear the current task to be notified
    task_to_notify = nullptr;

    // be sure to release the mutex
    unlock();

    // negative values mean an error
    Readings readings = {-1, -1, -1, -1, -1, -1};

    // if reading failed, return readings with negative values
    if (!readings_ok)
        return readings;

    // calculate final results as difference between 'off' and 'on' measurements
    // keep trnaslating reading number to the actual sensor measured
    int reading_num = 0;
    for (int ds_num = 0; ds_num < 6; ds_num++) {
        // update only the measured sensors
        if ((gpio_ex_sensors & spi::gpio::DISTANCE_SENSORS[ds_num]) != 0) {
            // calculate the difference, avoid unsigned overflows
            int new_val = static_cast<int>(readings_on[reading_num])
                - static_cast<int>(readings_off[reading_num]);

            // saturate to non-negative value
            readings.sensor[ds_num] = std::max(new_val, 0);

            // as this sensor was in measurements, advance the current measurement number
            reading_num++;
        }
    }

    return readings;
}

void notify_from_isr(bool &should_yield) {
    // as it is quite fast, just always disable the timer here
    HAL_TIM_Base_Stop_IT(&ds_htim);
    if (task_to_notify != nullptr)
        vTaskNotifyGiveFromISR(task_to_notify, reinterpret_cast<BaseType_t*>(&should_yield));
}

/*
 *
 * Model for the relation: distance(ADC_value)
 * Found experimentally based on measurements for front sensors (easy to measure by mobing back).
 *    D = a * x^b + c + d / (x + e)
 * Results: works quite bad, and is too complex.
 */
Distances Readings::to_distances() {
    Distances distances;
    constexpr float params[] = {-0.7097688, 0.0540759, 1.10880182, 4.98670003, 50.91384991};
    for (int i = 0; i < n_elements(sensor); i++) {
        if (sensor[i] < 0)
            distances.sensor[i] = -1;
        else {
#if 1
            float adc = sensor[i];
            float dist = 0;
            if (adc < 2)
                dist = 1.0; // we cannot see further than 1 meter anyway
            else if (adc >= 3600)
                dist = 0;
            else
                dist =
                    params[0] * std::pow(adc, params[1]) + params[2] + params[3] / (adc + params[4]);
            distances.sensor[i] = dist;
#endif
        }
    }
    return distances;
}

void Readings::to_linearised(float linearised[6]) {
    for (int i = 0; i < n_elements(sensor); i++) {
        if (sensor[i] < 0)
            linearised[i] = -1;
        else {
            // simple linearisation in the range from 600 to 3500
            float val = 248.5 / (sensor[i] + 1057) - 0.04;
            linearised[i] = std::max(0.0f, val);
        }
    }
}

// for each sensor, starting from sensor 0, check if it is in `sensors`, if so,
// then set this sensor as the next channel number; save the number of channels and return it
static int configure_adc_channels(uint8_t sensors) {
    // if no channels have been selected, we cannot configure ADC (SQR1:L[3:0] requires at least one!)
    if (sensors == 0)
        return 0;

    // channel number is equal to ds_num (sensor 0 -> channel 0, etc.)
    auto ds_num2channel_num = [] (int ds_num) { return ds_num; };

    static_assert(n_elements(spi::gpio::DISTANCE_SENSORS) <= 6,
            "This implementation writes to SQR3 only, which contains only 6 ADC conversions, \
            for more, we also have to use SQR2 and SQR1.");

    // reset all channels
    uint32_t new_SQR3 = 0;

    uint32_t current_channel_rank = 0;
    for (int ds_num = 0; ds_num < n_elements(spi::gpio::DISTANCE_SENSORS); ds_num++) {
        // for each sensor check if it has been chosen
        // the `sensors` variable contains distance sensors in bit format for GPIO expander
        if ((sensors & spi::gpio::DISTANCE_SENSORS[ds_num]) != 0) {
            // if so, configure its corresponding channel, set the channal number for the rank
            //   next channel (num 0..17) starts every 5 bits
            new_SQR3 |= ds_num2channel_num(ds_num) << (5 * current_channel_rank);

            // increase the next channel rank to be configured
            current_channel_rank++;
        }
    }

    // set the final number of ranks to be converted
    int n_conversions = current_channel_rank;
    int sqr_num_ranks = n_conversions - 1; // 0 means 1 conversion
    configASSERT(n_conversions > 0 && n_conversions <= n_elements(spi::gpio::DISTANCE_SENSORS));

    // set the same sampling times for all channels with distance sensors
    uint32_t new_SMPR2 = 0;
    for (int ds_num = 0; ds_num < n_elements(spi::gpio::DISTANCE_SENSORS); ds_num++) {
        // each channel has 3 bits for sampling time
        new_SMPR2 |= ADC_SAMPLING_TIME << (ds_num2channel_num(ds_num) * 3);
    }

    // update the actual registers
    ds_hadc.Instance->SMPR2 = new_SMPR2;
    ds_hadc.Instance->SQR3 = new_SQR3;
    ds_hadc.Instance->SQR1 = sqr_num_ranks << ADC_SQR1_L_Pos;

    // return number of conversions
    return n_conversions;
}

static bool wait_for_notification() {
    // task_to_notify must have been set before!
    auto result = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MAX_ADC_WAIT_TIME_MS));
    bool ok = result != 0;
    return ok;
}

static bool delay_us(int us) {
    if (us == 0) return true;
	ds_htim.Instance->ARR = us;
	ds_htim.Instance->CNT = 0;
    // do not use HAL to avoid wasting time for function call
    // bool ok = HAL_TIM_Base_Start_IT(&ds_htim) == HAL_OK;
    __HAL_TIM_ENABLE_IT(&ds_htim, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&ds_htim);
    bool ok = wait_for_notification();
    return ok;
}

static void lock() {
    // this should always succeed as we wait indefinitelly
    bool taken = xSemaphoreTake(adc_mutex, portMAX_DELAY) == pdPASS;
    configASSERT(taken);
}

static void unlock() {
    // this could only fail if we didn't take the semaphore earlier
    bool could_give = xSemaphoreGive(adc_mutex) == pdPASS;
    configASSERT(could_give);
}

} // namespace distance_sensors
