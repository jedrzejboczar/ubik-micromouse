/*
 * Implementation of microcontroller interrupts,
 * either directly, or using STM32HAL Callback functions.
 *
 * As each callback may be needed for different modules,
 * having all of them implemented in one place is probably
 * the best way to handle this problem. Each implementation
 * should just call a handle function from given module.
 */

#include "ubik/common/distance_sensors.h"


extern "C" {

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    bool should_yield = false;

    if (hadc->Instance == distance_sensors::ds_hadc.Instance) {
        distance_sensors::notify_from_isr(should_yield);
    }

    portYIELD_FROM_ISR(should_yield);
}

void callback_timer_period_elapsed(TIM_HandleTypeDef *htim) {
    bool should_yield = false;

    if (htim->Instance == distance_sensors::ds_htim.Instance) {
        distance_sensors::notify_from_isr(should_yield);
    }

    portYIELD_FROM_ISR(should_yield);
}

}

