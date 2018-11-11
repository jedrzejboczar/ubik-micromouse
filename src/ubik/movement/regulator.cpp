
#include <cstddef>
#include <algorithm>

#include "FreeRTOS.h"
#include "task.h"

#include "pid.h"
#include "motor_control.h"
#include "spi_devices.h"

#include "ubik/logging/logging.h"

namespace movement {
namespace regulator {

// cumulative positions for each wheel
// we need to keep track of these as the angle wraps to zero after max value
// the unit is the same as for encoder readings
// // for 12-bit resulution we can store ~53km of forward motion
static constexpr int32_t MAX_ENCODER_READING = (1 << 12) - 1;


static void update_position(int32_t &position, int32_t angle, int32_t last_angle);




void regulation_task(void *) {
    int32_t position_left = 0;
    int32_t position_right = 0;

    spi::initialise();
    movement::driver::initialise();

    float fs = 1e3;
    PID pid_left(fs), pid_right(fs);

    // get initial state (wheel angles)
    spi::EncoderReadings readings = spi::read_encoders();
    for (size_t i = 0; !readings.both_ok(); i++) {
        readings = spi::read_encoders();
        configASSERT(i < 10);
    }
    int32_t last_angle_left = readings.left.angle;
    int32_t last_angle_right = readings.right.angle;

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

        if (counter++ % 20 == 0)
            logging::printf(200, "totL = %8d, totR = %8d, angL = %5d, angR = %5d\n",
                    position_left, position_right, last_angle_left, last_angle_right);

        // update position
        //
        // calculate PID output
        //
        // set motors pulse
        //


        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(1));
    }
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


} // namespace regulator
} // namespace movement
