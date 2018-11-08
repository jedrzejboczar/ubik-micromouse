#include <algorithm>

// #include "ubik/maze/directions.h"

namespace movement {

struct Position {
    float x, y, theta;
};

class Controller {
    float dt;
    float dist_remaining;
    float vel_current;
    float vel_desired;
    float acc;
public:
    Controller(float dt): dt(dt), vel_current(0) {}

    // void move(Dir dir);

    void delay(float dt);
    void move_line(float distance, float vel_desired, float acc, float vel_final=0) {
        this->acc = acc;
        this->vel_desired = vel_desired;

        dist_remaining = distance;
        // TODO: probably better absolute or based on velocity
        // float margin = 0.01f * distance;
        float margin = 0.0;

        bool is_breaking = false;

        while (dist_remaining > 0 + margin) {
            if (!is_breaking && should_be_breaking(dist_remaining, vel_final)) {
                this->vel_desired = vel_final;
                is_breaking = true;
            }

            update_velocity(dt);

            // reduce remaining distance
            dist_remaining -= vel_current * dt;

            // wait
            delay(dt);
        }
    }

    // void movement_loop(float vel_final) {
    //     vel_current = 0;
    //     // dist_remaining =
    //
    //     // while ()
    // }
    //
    // void step(float dt) {
    // }

    bool should_be_breaking(float dist_remaining, float vel_final)  {
        /*
         * calculate the distance required not to overshoot the target,
         * based on current speed, acceleration and final speed in this move
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
        float dist_required = (vel_current + vel_final) * (vel_current - vel_final) / (2 * acc);
        return dist_remaining <= dist_required;
    }

    void update_velocity(float dt) {
        /*
         * Adjust current speed to obtaing desired trapezoidal shape,
         * where `acc` gives the speed of change:
         *     ^
         *     |        vel_desired__ __________
         * vel |                     /          \
         *     |                    /            \_____ __vel_final
         *     | vel_initial__ ____/
         *     +---------------------------------------------------->
         *                              time
         */
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
