#pragma once

#include <cstddef>
#include <cmath>
#include <algorithm>
#include <utility>
#include <tuple>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "pid.h"
#include "motor_control.h"
#include "spi_devices.h"
#include "robot_parameters.h"

namespace movement {
namespace regulator {


// TODO: better interface

void initialise();
void regulation_task(void *);

// this calls motors::set_enabled() but also handles internal regulator
// state correctly
void set_enabled(bool enabled);

// set the absolute value of current regulation target
void set_target(float translation_meters, float rotation_radians);
// modify the value of current regulation target by given amouts (more practical)
void update_target_by(float translation_meters, float rotation_radians);


} // namespace regulator


// regulator.cpp implements movement::update_target_by()
// this is to interface with controller.h, the function just calls
// movement::regulator::update_target_by()


} // namespace movement
