#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "ubik/common/robot_parameters.h"
#include "ubik/common/integrator.h"
#include "ubik/common/distance_sensors.h"
#include "ubik/logging/logging.h"
#include "ubik/movement/regulator.h"


namespace movement::correction {

std::pair<float, float> next_position_delta(float dt);

namespace side_walls {
bool calibrate();
void set_enabled(bool enabled);
} // namespace side_walls

namespace front_walls {
bool calibrate();
void set_enabled(bool enabled);
} // namespace front_walls


} // namespace movement::correction
