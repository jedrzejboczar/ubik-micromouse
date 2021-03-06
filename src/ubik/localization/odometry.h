#pragma once

#include <tuple>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "ubik/common/spi_devices.h"
#include "ubik/common/robot_parameters.h"

/*
 * This module is responsible for encoders odometry (and later
 * for I2C too).
 * It does not have any task for measurements, the corresponding
 * functions should probably be called somewhere else.
 */


namespace localization::odometry {

static constexpr int32_t MAX_ENCODER_READING = (1 << 12) - 1;

struct Position {
    float x, y, theta;
};


void initialise();

/*
 * Resets last stored wheel angles to the current values.
 * This is needed in order to avoid drasticall position changes
 * after longer periods of not-reading encoders.
 * Should be called when PID regulation is started.
 *
 * (!) This may fail if the encoders cannot be read (tries more
 * than once).
 */
bool reset_wheels_state();

// This module is only for handling the localization logic and storing
// data from different sources, so encoders must be read somewhere else
// It is natural to do this in the regulation loop.
bool next_encoders_reading();

// returns pair (left, right)
// this returns the raw, internal representation, as it has always the
// finest resoltion, other function use floats to avoid calculation problems
std::pair<int32_t, int32_t> get_cumulative_encoder_ticks();
// returns current position (x, y, theta)
Position get_current_position();
void set_current_position(Position pos);

 // Convert between translation/rotation ant angle of the wheels in encoder ticks.
std::pair<float, float> convert_to_encoder_ticks(float translation_meters, float rotation_radians);
std::pair<float, float> convert_to_translation_rotation(float left_angle, float right_angle);



} // namespace localization::odometry


