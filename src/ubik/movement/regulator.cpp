
#include <cstddef>
#include <cmath>
#include <algorithm>
#include <utility>
#include <tuple>

#include "FreeRTOS.h"
#include "task.h"

#include "pid.h"
#include "motor_control.h"
#include "spi_devices.h"
#include "robot_parameters.h"

#include "ubik/logging/logging.h"

namespace movement {
namespace regulator {

// cumulative positions for each wheel
// we need to keep track of these as the angle wraps to zero after max value
// the unit is the same as for encoder readings
// // for 12-bit resulution we can store ~53km of forward motion
static constexpr int32_t MAX_ENCODER_READING = (1 << 12) - 1;
static constexpr float   LOOP_FREQUENCY = 1e3;


static void update_position(int32_t &position, int32_t angle, int32_t last_angle);
static std::pair<int32_t, int32_t>
    convert_to_encoder_ticks(float translation_meters, float rotation_radians);

static int32_t set_point_left = 0;
static int32_t set_point_right = 0;



void regulation_task(void *) {
    int32_t position_left = 0;
    int32_t position_right = 0;

    // prepare PID regulators
    PID pid_left(LOOP_FREQUENCY), pid_right(LOOP_FREQUENCY);
    pid_left .set_params(3e-5, .1e-6/* , .1e-6 */);
    pid_right.set_params(3e-5, .1e-6/* , .1e-6 */);

    // get initial state (wheel angles)
    spi::EncoderReadings readings = spi::read_encoders();
    for (size_t i = 0; !readings.both_ok(); i++) {
        readings = spi::read_encoders();
        configASSERT(i < 10);
    }
    int32_t last_angle_left = readings.left.angle;
    int32_t last_angle_right = readings.right.angle;

    // prepare loop timing
    auto last_start = xTaskGetTickCount();
    size_t counter = 0;

    while (1) {
        // read encoders
        readings = spi::read_encoders();

        // accumulate wheel positions and save last angles
        if (readings.valid) {
            // left wheel readings decreases in forward direction, so negate it
            if (readings.left.is_ok()) {
                update_position(position_left, -readings.left.angle, -last_angle_left);
                last_angle_left = readings.left.angle;
            }
            // right wheel readings increase in forward direction
            if (readings.right.is_ok()) {
                update_position(position_right, readings.right.angle, last_angle_right);
                last_angle_right = readings.right.angle;
            }
        }

        // calculate PID output
        float out_left = pid_left.next(set_point_left, position_left);
        float out_right = pid_right.next(set_point_right, position_right);

        // normalize output so that we won't have to change parameters max_pulse changes
        // normalize such that PID output = 1.0 corresponds to 100% pulse
        out_left = out_left * motors::max_pulse();
        out_right = out_right * motors::max_pulse();

        // calculate motor directions and pulse
        motors::set_direction(
                out_left  > 0 ? FORWARDS : BACKWARDS,
                out_right > 0 ? FORWARDS : BACKWARDS);
        motors::set_pulse(std::abs(out_left), std::abs(out_right));

        if (counter++ % 50 == 0)
            logging::printf(120, "%8d,%8d,%8d,%8d,%8d,%8d\n",
                    (int) set_point_left, position_left, (int) out_left,
                    (int) set_point_right, position_right, (int) out_right);
            // logging::printf(120, "%ld,%ld,%ld,%ld\n",
            //         (int) set_point_left, (int) out_left, (int) set_point_right, (int) out_right);
            // logging::printf(120, "spl=%ld,outl=%ld,spr=%ld,outr=%ld\n",
            //         static_cast<int>(set_point_left), out_left, static_cast<int>(set_point_right), out_right);

        // TODO: use a timer instead, for a higher frequency
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(1));
    }
}

void set_regulation_target(float translation_meters, float rotation_radians) {
    int32_t ticks_left, ticks_right;
    std::tie(ticks_left, ticks_right)
        = convert_to_encoder_ticks(translation_meters, rotation_radians);

    // TODO: mutex, lock global variables!
    set_point_left = ticks_left;
    set_point_right = ticks_right;
}

void update_regulation_target(float translation_meters, float rotation_radians) {
    int32_t ticks_left, ticks_right;
    std::tie(ticks_left, ticks_right)
        = convert_to_encoder_ticks(translation_meters, rotation_radians);

    // TODO: mutex, lock global variables!
    set_point_left += ticks_left;
    set_point_right += ticks_right;
}


void update_position(int32_t &position, int32_t angle, int32_t last_angle) {
    int32_t angle_delta = angle - last_angle;
    // detect the right direction of movement
    // if delta is greater than half turn, we assume it is movement in other direction
    if (std::abs(angle_delta) > MAX_ENCODER_READING / 2) {
        if (angle_delta >= 0)
            angle_delta -= MAX_ENCODER_READING / 2;
        else
            angle_delta += MAX_ENCODER_READING / 2;
    }
    // integrate the position delta to obtain cumulative position
    position += angle_delta;
}

/*
 * Convert from translation/rotation to the total angle of the wheels.
 *    v_lin = (v_right + v_left) / 2
 *    v_ang = (v_right - v_left) / 2
 * so:
 *    v_left = v_lin - v_ang
 *    v_right = v_lin + v_ang
 * and this (for velocity) also translates to distance.
 * About the direction:
 *    When v_ang is positive, we turn left, looking from top.
 *    This is the same as with positive angle in Euclidean coordinates system.
 */
std::pair<int32_t, int32_t> convert_to_encoder_ticks(float translation_meters, float rotation_radians)
{
    // convert trans/rot to distance traveled by each wheel
    float distance_left_m = translation_meters - constants::rotation_angle2arc_length(rotation_radians);
    float distance_right_m = translation_meters + constants::rotation_angle2arc_length(rotation_radians);
    // convert distance of the wheel to number of encoder turns
    // (encoders are after gear transmission, so they turn much faster)
    float n_turns_left = distance_left_m / constants::WHEEL_CIRCUMFERENCE * constants::GEAR_RATIO;
    float n_turns_right = distance_right_m / constants::WHEEL_CIRCUMFERENCE * constants::GEAR_RATIO;
    // convert number of turns to encoder ticks
    float n_ticks_left = n_turns_left * MAX_ENCODER_READING;
    float n_ticks_right = n_turns_right * MAX_ENCODER_READING;
    // round
    int32_t ticks_left = std::lround(n_ticks_left);
    int32_t ticks_right = std::lround(n_ticks_right);
    return std::make_pair(ticks_left, ticks_right);
}



} // namespace regulator
} // namespace movement
