#pragma once

#include <algorithm>

#define DEBUG_STORE_VEL_DESIRED_AND_ACC 1


namespace movement::controller {

/*
 * Interface functions definitions.
 *
 * Controller requires implementation of a function that will update
 * regulation set point (position) by the fiven amount (meters, radians).
 * Also a delay is required, the dt parameter depends on the contrtoller
 * frequency specified.
 */
void update_target_by(float distance_linear, float distance_angular);
void delay(float dt);


/* Simple struct for storing both linear and angluar motion data.  */
struct Vec2 {
    float lin;
    float ang;

    friend Vec2 abs(Vec2 vec) {
        vec.lin = std::abs(vec.lin);
        vec.ang = std::abs(vec.ang);
        return vec;
    }
};

void set_frequency(float frequency);

/*
 * Move either in a straight line or by rotating in place.
 * Robot will try to reach `vel_desired`, but the movement will be restricted
 * by the given `acc`eleration. When needed, the robot will start breaking,
 * to reach `vel_final` at the end of movement.
 * Note: `vel_final` may not be reached if this is impossible, e.g. `vel_current`
 * at the beggining of the movement is so big, that `vel final` cannot be reached
 * at the given `acc`.
 *
 * These functions assume (and set) current velocity of opposite movement to be zero,
 * e.g. calling move_line() resets `vel_current.ang` to zero, so that it can be
 * used after move_arc() and won't result in rong behaviour.
 * If current velocity is non-zero, use proper functions to reduce it before.
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



} // namespace movement::controller
