#pragma once

#include <algorithm>

#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "spi_devices.h"

namespace distance_sensors {

/*
 * There are 6 sensors. Sensors (in the software) are numerated from 0 to 5.
 */

// negative values signalize measurement errors, or not-requrested sensors
struct Distances {
    float sensor[6];
};
struct Readings {
    int16_t sensor[6];
    Distances to_distances(); // TODO
};



void initialise();

// use gpio::DISTANCE_SENSORS() to choose bits corresponding to the sensors measured
// do not use this function call after call (it needs at least 100us between calls
// for good results - LEDs and phototransistors must fully turn off)
// IMPORTANT: this function uses FreeRTOS direct to task notifications, taking
// the current task's handle!
static constexpr int LEDS_TURN_ON_WAIT_TIME_US = 100;
Readings read(uint8_t gpio_ex_sensors);


// have to be called in interrupts from peripherals used:
// ADC conversion complete interrupt and timer update (period elapsed) interrupt
extern ADC_HandleTypeDef &ds_hadc;
extern TIM_HandleTypeDef &ds_htim;
void notify_from_isr(bool &should_yield);


} // namespace distance_sensors
