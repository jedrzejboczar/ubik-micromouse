#pragma once

#include <algorithm>

#include "robot_parameters.h"

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
     * Move either in a straight line or by rotating in place.
     * Robot will try to reach `vel_desired`, but the movement will be restricted
     * by the given `acc`eleration. When needed, the robot will start breaking,
     * to reach `vel_final` at the end of movement.
     * Note: `vel_final` may not be reached if this is impossible, e.g. `vel_current`
     * at the beggining of the movement is so big, that `vel final` cannot be reached
     * at the given `acc`.
     *
     * All values are in SI system (meters, radians, meters/s, etc.).
     * Positive angle means turning left (as a directed angle in XY coordinate
     * system).
     */
    void move_line(float distance, float vel_desired, float acc, float vel_final = 0);
    void move_rotate(float angle, float vel_desired, float acc, float vel_final = 0);

    /*
     * Moving along an arc puts restrictions on the movement, so `vel_desired`, `acc`
     * and `vel_final` refer to the linear motion parameters.
     *
     * The restriction is:
     *    constant curvature of an arc: k = d_angle / d_s = 1 / R = const
     * So:
     *    k = 1 / R
     *    d_angle = d_s * k
     *    vel_ang = vel_lin * k
     * From properites of an arc:
     *    length of an arc: s = angle * R
     *                      R = s / angle
     * So:
     *    k = angle / s
     *    vel_ang = vel_lin * k
     * Where:
     *    k - curvature of the arc
     *    vel_lin - the current linear velocity from the profiler
     *    angle - `angle` argument, the angle of the arc
     *    s - `distance` argument, the length of the arc
     *
     * Important:
     *   distance - Vec2{distance_lin, angle}
     *   All arguemnts besides vel_final shall be non-zero.
     *   Only `distance` and `angle` may be negative (for other arguments sign is ignored).
     *
     * TODO: this restriction can be removed by auto-calling move_line() / move_rotate()
     *   distance - must be non-zero!
     */
    void move_arc(Vec2 distance, float vel_desired, float acc, float vel_final = 0);

    /*
     * (!) This should generally not be used (!)
     * It does no coordination of linear and angular movement, so one of them can end first
     * which means that part of the movement will be some-kind-of-arc-like-trajectory and
     * the rest will be either a straight line or turning in place.
     * Use `move_line()` and `move_rotate()` instead.
     *
     * TODO: try recalculting acc to increase it if really needed
     */
    void move_lin_ang(Vec2 distance, Vec2 vel_desired, Vec2 acc, Vec2 vel_final = Vec2{0, 0});

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
