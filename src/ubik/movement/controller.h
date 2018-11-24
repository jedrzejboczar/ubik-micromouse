#pragma once

#include <algorithm>

#define DEBUG_STORE_VEL_DESIRED_AND_ACC 1


namespace movement {

// Controller requires implementation of a function that will update
// regulation set point (position) by the fiven amount (meters, radians).
void update_target_by(float distance_linear, float distance_angular);


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

#if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
    Vec2 acc;
    Vec2 vel_desired;
#endif

public:
    Controller(float frequency):
        dt(1.0f / frequency),
        dist_remaining{0, 0},
        vel_current{0, 0}
    {  }

    void delay(float dt);

    /*
     * Moves robot along an arc described by the distance.
     * Movement is restricted by the acceleration, desired velocity and final velocity.
     * All arguments are taken by their absolute value, only the distance's sign determines
     * movement direction.
     * Each Vec2 consists of linear and angular coordinate.
     * Units are from SI table, so:
     *    linear units: meter, meter/s, ...
     *    angular units: radian, radian/s, ...
     *    angular direction is positive when robot turns left
     */
    void move_arc(Vec2 distance, Vec2 vel_desired, Vec2 acc, Vec2 vel_final=Vec2{0, 0});

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
    bool should_be_breaking(float dist_remaining, float vel_current, float vel_final, float acc);
    float required_breaking_acc(float dist_remaining, float vel_current, float vel_final);

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
     *
     * Theoretically, the integration should be ideal, as we use only
     * constant accelerations anyway.
     */
    float update_velocity(float vel_current, float vel_desired, float acc);
};


} // namespace movement
