#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "ubik/common/robot_parameters.h"
#include "ubik/common/distance_sensors.h"
#include "ubik/logging/logging.h"
#include "ubik/movement/regulator.h"

namespace movement::correction {

std::pair<float, float> calculate_correction_velocities();

namespace side_walls {
bool calibrate();
void set_enabled(bool enabled);
} // namespace side_walls

namespace front_walls {
bool calibrate();
void set_enabled(bool enabled);
} // namespace front_walls


} // namespace movement::correction
