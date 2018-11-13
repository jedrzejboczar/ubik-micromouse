#pragma once

#include <algorithm>

// #include "ubik/maze/directions.h"

#include "FreeRTOS.h"
#include "task.h"

#include "regulator.h"

#include "ubik/logging/logging.h"

namespace movement {

// struct Position {
//     float x, y, theta;
// };

class Controller {
    float dt;
    float dist_remaining;
    float vel_current;
    float vel_desired;
    float acc;
public:
    // TODO: remove this by using separate variables for linear/angular motion
    void reset() {
        dist_remaining = vel_current = vel_desired = acc = 0;
    }

    Controller(float frequency):
        dt(1.0f / frequency), dist_remaining(0), vel_current(0), vel_desired(0), acc(0) {}

    // void move(Dir dir);

    void delay(float dt) {
        vTaskDelay(pdMS_TO_TICKS(1e3f * dt));
    }

    // move by given distance, try to achieve vel_desired with given acc, end with vel_final
    // all input values should be positive (absolute values are taken)
    void move_line(float distance, float vel_desired, float acc, float vel_final=0) {
        int direction = distance > 0 ? 1 : -1;

        // use only absolute values, direction only affects the resulting velocities sign
        distance = std::abs(distance);
        vel_desired = std::abs(vel_desired);
        acc = std::abs(acc);
        vel_final = std::abs(vel_final);

        this->acc = acc;
        this->vel_desired = vel_desired;
        this->dist_remaining = distance;

        bool is_breaking = false;
        while (dist_remaining > vel_current * dt) {
            if (!is_breaking && should_be_breaking(dist_remaining, vel_final)) {
                this->vel_desired = vel_final;
                is_breaking = true;
            }

            // reduce remaining distance
            float distance_traveled = vel_current * dt;
            dist_remaining -= distance_traveled;

            // update after moving to avoid deadlock
            update_velocity();

            regulator::update_target_by(direction * distance_traveled, 0);

            // wait
            delay(dt);
        }

        // compensate for the last step if any distance is still remaining (t = s/v)
        dist_remaining = 0;
        delay(dist_remaining / vel_current);
    }

    // all in radians, radians per sec, etc.
    void move_turn(float angle, float vel_desired, float acc, float vel_final=0) {
        int direction = angle > 0 ? 1 : -1;

        // use only absolute values, direction only affects the resulting velocities sign
        angle = std::abs(angle);
        vel_desired = std::abs(vel_desired);
        acc = std::abs(acc);
        vel_final = std::abs(vel_final);

        this->acc = acc;
        this->vel_desired = vel_desired;
        this->dist_remaining = angle;

        bool is_breaking = false;
        while (dist_remaining > vel_current * dt) {
            if (!is_breaking && should_be_breaking(dist_remaining, vel_final)) {
                this->vel_desired = vel_final;
                is_breaking = true;
            }

            // reduce remaining distance
            float distance_traveled = vel_current * dt;
            dist_remaining -= distance_traveled;

            // update after moving to avoid deadlock
            update_velocity();

            // distance_traveled is in radians
            regulator::update_target_by(0, direction * distance_traveled);

            // wait
            delay(dt);
        }

        // compensate for the last step if any distance is still remaining (t = s/v)
        dist_remaining = 0;
        delay(dist_remaining / vel_current);

    }

    /*
     * Calculates the distance required not to overshoot the target,
     * based on current speed, acceleration and final speed in this move.
     *
     * s = vt - at^2 / 2
     * t = dv / a        = (vc - vf) / a
     *
     * s = vc * (vc - vf) / a - a * [ (vc - vf) / a ]^2 / 2
     * s = vc * (vc - vf) / a - (vc - vf)^2 / 2a
     * s = 2(vc^2 - vc*vf) / 2a - (vc^2 - 2vc*vf + vf^2) / 2a
     * s = (vc^2 - vf^2) / 2a
     * s = (vc + vf)(vc - vf) / 2a
     */
    bool should_be_breaking(float dist_remaining, float vel_final)  {
        float dist_required = (vel_current + vel_final) * (vel_current - vel_final) / (2 * acc);
        return dist_remaining <= dist_required;
    }

    /*
     * Adjust `vel_current` to obtaing desired trapezoidal shape,
     * where `acc` gives the speed of change:
     *     ^
     *     |        vel_desired__ __________
     * vel |                     /          \
     *     |                    /            \_____ __vel_final
     *     | vel_initial__ ____/
     *     +---------------------------------------------------->
     *                              time
     */
    void update_velocity() {
        if (vel_current < vel_desired) {
            vel_current += acc * dt;
            vel_current = std::min(vel_current, vel_desired); // remove overshoot
        } else if (vel_current > vel_desired) {
            vel_current -= acc * dt;
            vel_current = std::max(vel_current, vel_desired); // remove overshoot
        }
    }
};


} // namespace movement
