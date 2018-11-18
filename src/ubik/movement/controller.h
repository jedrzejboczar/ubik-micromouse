#pragma once

#include <algorithm>

// #include "ubik/maze/directions.h"

// #include "regulator.h"

// #include "ubik/logging/logging.h"

namespace movement {


struct Vec2 {
    float lin;
    float ang;

    friend Vec2 abs(Vec2 vec) {
        vec.lin = std::abs(vec.lin);
        vec.ang = std::abs(vec.ang);
        return vec;
    }
};


class Controller {
    float dt;
    Vec2 dist_remaining;
    Vec2 vel_current;

    Vec2 acc;
    Vec2 vel_desired;
public:
    Controller(float frequency):
        dt(1.0f / frequency),
        dist_remaining{0, 0},
        vel_current{0, 0}
    {  }

    // void move(Dir dir);

    void delay(float dt);

    // all in radians, radians per sec, etc.
    void move_arc(Vec2 distance, Vec2 vel_desired, Vec2 acc, Vec2 vel_final=Vec2{0, 0}) {
        int direction_lin = distance.lin > 0 ? 1 : -1;
        int direction_ang = distance.ang > 0 ? 1 : -1;

        // use only absolute values, direction only affects the resulting velocities sign
        distance = abs(distance);
        vel_desired = abs(vel_desired);
        acc = abs(acc);
        vel_final = abs(vel_final);

        // for debugging
        this->vel_desired = vel_desired;
        this->acc = acc;

        // save the remaining distance to a member variable
        dist_remaining = distance;

        // determine if one of the movements is not being used
        // this is needed for breaking calculations to avoid division by zero
        bool is_lin = distance.lin > 0 && vel_desired.lin > 0 && acc.lin > 0;
        bool is_ang = distance.ang > 0 && vel_desired.ang > 0 && acc.ang > 0;

        // keep track if breaking has been started
        bool is_breaking_lin = false;
        bool is_breaking_ang = false;

        while ( (is_lin && dist_remaining.lin > vel_current.lin * dt)
                || (is_ang && dist_remaining.ang > vel_current.ang * dt) )
        // while ( (is_lin && dist_remaining.lin > 0)
        //         || (is_ang && dist_remaining.ang > 0) )
        {
            // if we have to start breaking, set the desired velocity to the final one
            if (is_lin && !is_breaking_lin && should_be_breaking(
                        dist_remaining.lin, vel_current.lin, vel_final.lin, acc.lin))
            {
                vel_desired.lin = vel_final.lin;
                is_breaking_lin = true;
                acc.lin = required_breaking_acc(dist_remaining.lin, vel_current.lin, vel_final.lin);
                this->acc.lin = required_breaking_acc(dist_remaining.lin, vel_current.lin, vel_final.lin);
            }
            if (is_ang && !is_breaking_ang && should_be_breaking(
                        dist_remaining.ang, vel_current.ang, vel_final.ang, acc.ang))
            {
                vel_desired.ang = vel_final.ang;
                is_breaking_ang = true;
                acc.ang = required_breaking_acc(dist_remaining.ang, vel_current.ang, vel_final.ang);
                this->acc.ang = required_breaking_acc(dist_remaining.ang, vel_current.ang, vel_final.ang);
            }

            // for debugging
            this->vel_desired = vel_desired;

            // calculate new velocity
            float vel_new_lin = update_velocity(vel_current.lin, vel_desired.lin, acc.lin);
            float vel_new_ang = update_velocity(vel_current.ang, vel_desired.ang, acc.ang);

            // calculate the distance traveled using trapezoidal integration
            // (which should be "ideal" for constant accelerations)
            // trapezoidal rule: (a+b) / 2 * h
            float distance_traveled_lin = (vel_current.lin + vel_new_lin) / 2 * dt;
            float distance_traveled_ang = (vel_current.ang + vel_new_ang) / 2 * dt;

            // reduce remaining distance
            dist_remaining.lin -= distance_traveled_lin;
            dist_remaining.ang -= distance_traveled_ang;

            // distance_traveled is in radians
            // regulator::update_target_by(
            //         direction_lin * distance_traveled_lin,
            //         direction_ang * distance_traveled_ang);

            // save the new velocity as current
            vel_current = {vel_new_lin, vel_new_ang};

            // wait
            delay(dt);
        }

        // compensate for the last step if any distance is still remaining (t = s/v)
        // TODO: this may create errors if both velocities are non-zero
        float t_lin = (vel_current.lin > 0) ? (dist_remaining.lin / vel_current.lin) : 0;
        float t_ang = (vel_current.ang > 0) ? (dist_remaining.ang / vel_current.ang) : 0;
        delay(std::max(t_lin, t_ang));
        dist_remaining.lin = dist_remaining.ang = 0;

        // compensate for current velocity being other than requested
        // (we can, as we supply position, not velocity to the regulator)
        vel_current = vel_final;
    }

    /*
     * Calculates the distance required not to overshoot the target,
     * based on current speed, acceleration and final speed in this move.
     *
     *    s = vt - at^2 / 2;
     *    t = dv / a        = (vc - vf) / a
     *
     *    s = vc * (vc - vf) / a - a * [ (vc - vf) / a ]^2 / 2
     *    s = vc * (vc - vf) / a - (vc - vf)^2 / 2a
     *    s = 2(vc^2 - vc*vf) / 2a - (vc^2 - 2vc*vf + vf^2) / 2a
     *    s = (vc^2 - vf^2) / 2a
     *    s = (vc + vf)(vc - vf) / 2a
     *
     * For the required breaking acceleration it is just:
     *    a = (vc + vf)(vc - vf) / 2s
     */
    bool should_be_breaking(float dist_remaining, float vel_current, float vel_final, float acc)  {
        float dist_required = (vel_current + vel_final) * (vel_current - vel_final) / (2 * acc);
        return dist_remaining <= dist_required;
    }
    float required_breaking_acc(float dist_remaining, float vel_current, float vel_final)  {
        float acc = (vel_current + vel_final) * (vel_current - vel_final) / (2 * dist_remaining);
        return acc;
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
    // the integration should be ideal, as we use only constant accelerations
    float update_velocity(float vel_current, float vel_desired, float acc) {
        if (vel_current < vel_desired) {
            vel_current += acc * dt;
            vel_current = std::min(vel_current, vel_desired); // remove overshoot
        } else if (vel_current > vel_desired) {
            vel_current -= acc * dt;
            vel_current = std::max(vel_current, vel_desired); // remove overshoot
        }
        return vel_current;
    }
};


} // namespace movement
