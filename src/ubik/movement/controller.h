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


struct Vec2 {
    float lin;
    float ang;

    // // https://en.cppreference.com/w/cpp/language/operators : Binary arithmetic operators
    // Vec2& operator+=(const Vec2& rhs) {
    //     lin += rhs.lin;
    //     ang += rhs.ang;
    //     return *this;
    // }
    // friend Vec2 operator+(Vec2 lhs, const Vec2& rhs) {
    //     lhs += rhs;
    //     return lhs;
    // }
    // Vec2& operator-=(const Vec2& rhs) {
    //     lin -= rhs.lin;
    //     ang -= rhs.ang;
    //     return *this;
    // }
    // friend Vec2 operator-(Vec2 lhs, const Vec2& rhs) {
    //     lhs -= rhs;
    //     return lhs;
    // }
    // Vec2& operator*=(const Vec2& rhs) {
    //     lin *= rhs.lin;
    //     ang *= rhs.ang;
    //     return *this;
    // }
    // friend Vec2 operator*(Vec2 lhs, const Vec2& rhs) {
    //     lhs *= rhs;
    //     return lhs;
    // }
    // Vec2& operator/=(const Vec2& rhs) {
    //     lin /= rhs.lin;
    //     ang /= rhs.ang;
    //     return *this;
    // }
    // friend Vec2 operator/(Vec2 lhs, const Vec2& rhs) {
    //     lhs /= rhs;
    //     return lhs;
    // }
    // friend Vec2 abs(Vec2 vec) {
    //     vec.lin = std::abs(vec.lin);
    //     vec.ang = std::abs(vec.ang);
    //     return vec;
    // }

};



class Controller {
    float dt;
    Vec2 dist_remaining;
    Vec2 vel_current;
    Vec2 vel_desired;
    Vec2 acc;
public:
    // TODO: remove this by using separate variables for linear/angular motion
    void reset() {
        dist_remaining.lin = vel_current.lin = vel_desired.lin = acc.lin = 0;
        dist_remaining.ang = vel_current.ang = vel_desired.ang = acc.ang = 0;
    }

    Controller(float frequency):
        dt(1.0f / frequency),
        dist_remaining{0, 0}, vel_current{0, 0}, vel_desired{0, 0}, acc{0, 0} {}

    // void move(Dir dir);

    void delay(float dt) {
        vTaskDelay(pdMS_TO_TICKS(1e3f * dt));
    }

    // all in radians, radians per sec, etc.
    void move_arc(Vec2 distance, Vec2 vel_desired, Vec2 acc, Vec2 vel_final=Vec2{0, 0}) {
        int direction_lin = distance.lin > 0 ? 1 : -1;
        int direction_ang = distance.ang > 0 ? 1 : -1;

        // use only absolute values, direction only affects the resulting velocities sign
        distance.lin = std::abs(distance.lin);
        distance.ang = std::abs(distance.ang);
        vel_desired.lin = std::abs(vel_desired.lin);
        vel_desired.ang = std::abs(vel_desired.ang);
        acc.lin = std::abs(acc.lin);
        acc.ang = std::abs(acc.ang);
        vel_final.lin = std::abs(vel_final.lin);
        vel_final.ang = std::abs(vel_final.ang);

        // save the values for other functions
        this->acc = acc;
        this->vel_desired = vel_desired;
        this->dist_remaining = distance;

        bool is_breaking_lin = false;
        bool is_breaking_ang = false;
        while (dist_remaining.lin > vel_current.lin * dt
                || dist_remaining.ang > vel_current.ang * dt)
        {
            // if we have to start breaking, set the desired velocity to the final one
            if (!is_breaking_lin && should_be_breaking(
                        dist_remaining.lin, vel_current.lin, vel_final.lin, acc.lin))
            {
                this->vel_desired.lin = vel_final.lin;
                is_breaking_lin = true;
            }
            if (!is_breaking_ang && should_be_breaking(
                        dist_remaining.ang, vel_current.ang, vel_final.ang, acc.ang))
            {
                this->vel_desired.ang = vel_final.ang;
                is_breaking_ang = true;
            }

            // reduce remaining distance
            float distance_traveled_lin = vel_current.lin * dt;
            float distance_traveled_ang = vel_current.ang * dt;
            dist_remaining.lin -= distance_traveled_lin;
            dist_remaining.ang -= distance_traveled_ang;

            // update after moving to avoid deadlock
            update_velocity(vel_current.lin, vel_desired.lin, acc.lin);
            update_velocity(vel_current.ang, vel_desired.ang, acc.ang);

            // distance_traveled is in radians
            regulator::update_target_by(direction_lin * distance_traveled_lin,
                    direction_ang * distance_traveled_ang);

            // wait
            delay(dt);
        }

        // compensate for the last step if any distance is still remaining (t = s/v)
        dist_remaining.lin = dist_remaining.ang = 0;
        float t_lin = dist_remaining.lin / vel_current.lin;
        float t_ang = dist_remaining.ang / vel_current.ang;
        delay(std::max(t_lin, t_ang));

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
    bool should_be_breaking(float dist_remaining, float vel_current, float vel_final, float acc)  {
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
    void update_velocity(float &vel_current, float vel_desired, float acc) {
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
