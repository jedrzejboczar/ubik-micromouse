#include "controller.h"

/*
 * Higher level abstraction over movement trajectory generation.
 * Requires a regulator that will controll the motors to hold the
 * given set point positon.
 *
 * These functions are not thread safe at all!!!
 * There is only one robot and one controller,
 * just don't create artificial problems.
 */

namespace movement::controller {

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
static bool should_be_breaking(float dist_remaining, float vel_current, float vel_final, float acc);
// static float required_breaking_acc(float dist_remaining, float vel_current, float vel_final);

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
static float update_velocity(float vel_current, float vel_desired, float acc);



/* Controller state variables */
static float _dt = 0;
static Vec2 _dist_remaining = {0, 0};
static Vec2 _vel_current = {0, 0};
#if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
static Vec2 _acc = {0, 0};
static Vec2 _vel_desired = {0, 0};
#endif

void set_frequency(float frequency) {
    _dt = 1.0f / frequency;
}

void move_line(float distance, float vel_desired, float acc, float vel_final) {
    _vel_current.ang = 0;
    move_lin_ang({distance, 0}, {vel_desired, 0}, {acc, 0}, {vel_final, 0});
}
void move_rotate(float angle, float vel_desired, float acc, float vel_final) {
    _vel_current.lin = 0;
    move_lin_ang({0, angle}, {0, vel_desired}, {0, acc}, {0, vel_final});
}

void move_arc(Vec2 distance, float vel_desired, float acc, float vel_final) {
    int direction_lin = distance.lin > 0 ? 1 : -1;
    int direction_ang = distance.ang > 0 ? 1 : -1;

    // use only absolute values, direction only affects the resulting velocities sign
    distance = abs(distance);
    vel_desired = std::abs(vel_desired);
    acc = std::abs(acc);
    vel_final = std::abs(vel_final);

#if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
    _vel_desired.lin = vel_desired;
    _acc.lin = acc;
#endif

    // calculate the curvature of this arc, once as it is constant
    const float k = distance.ang / distance.lin;

    // save the remaining distance to a member variable
    _dist_remaining.lin = distance.lin;
    bool is_breaking = false;

    while ( _dist_remaining.lin > 0 )
    {

        // if we have to start breaking, set the desired velocity to the final one
        if (!is_breaking && should_be_breaking(
                    _dist_remaining.lin, _vel_current.lin, vel_final, acc))
        {
            vel_desired = vel_final;
            is_breaking = true;

            // TODO: for now do not do this, as it may cause a deadlock
            //             acc.lin = required_breaking_acc(_dist_remaining.lin, _vel_current.lin, vel_final.lin);
            // #if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
            //             _acc.lin = acc.lin;
            // #endif
        }

#if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
        _vel_desired.lin = vel_desired;
#endif

    // calculate new linear velocity
    float vel_new_lin = update_velocity(_vel_current.lin, vel_desired, acc);
    // calculate the angular velocity based on linear velocity
    float vel_new_ang = vel_new_lin * k;

    // calculate the distance traveled using trapezoidal integration
    // (which should be "ideal" for constant accelerations)
    // trapezoidal rule: (a+b) / 2 * h
    float distance_traveled_lin = (_vel_current.lin + vel_new_lin) / 2 * _dt;
    float distance_traveled_ang = (_vel_current.ang + vel_new_ang) / 2 * _dt;

    // reduce remaining distance
    _dist_remaining.lin -= distance_traveled_lin;
    _dist_remaining.ang -= distance_traveled_ang;

    // update regulator target
    update_target_by(
            direction_lin * distance_traveled_lin,
            direction_ang * distance_traveled_ang);

    // save the new velocity as current
    _vel_current = {vel_new_lin, vel_new_ang};

    // wait
    delay(_dt);

    }

    // compensate for the last step if any distance is still remaining (t = s/v)
    // TODO: this may create errors if both velocities are non-zero
    float t = (_vel_current.lin > 0) ? (_dist_remaining.lin / _vel_current.lin) : 0;
    delay(t);
    _dist_remaining = {0, 0};

    // compensate for current velocity if we couldn't break fast enough
    // (we can, as we supply position, not velocity to the regulator)
    if (_vel_current.lin > vel_final) {
        _vel_current.lin = vel_final;
        _vel_current.ang = _vel_current.lin * k;
    }
}


void move_lin_ang(Vec2 distance, Vec2 vel_desired, Vec2 acc, Vec2 vel_final) {
    int direction_lin = distance.lin > 0 ? 1 : -1;
    int direction_ang = distance.ang > 0 ? 1 : -1;

    // use only absolute values, direction only affects the resulting velocities sign
    distance = abs(distance);
    vel_desired = abs(vel_desired);
    acc = abs(acc);
    vel_final = abs(vel_final);

#if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
    _vel_desired = vel_desired;
    _acc = acc;
#endif

    // save the remaining distance to a member variable
    _dist_remaining = distance;

    // determine if one of the movements is not being used
    // this is needed for breaking calculations to avoid division by zero
    bool is_lin = distance.lin > 0 && vel_desired.lin > 0 && acc.lin > 0;
    bool is_ang = distance.ang > 0 && vel_desired.ang > 0 && acc.ang > 0;

    // keep track if breaking has been started
    bool is_breaking_lin = false;
    bool is_breaking_ang = false;

    // while ( (is_lin && dist_remaining.lin > vel_current.lin * dt)
    //         || (is_ang && dist_remaining.ang > vel_current.ang * dt) )
    while ( (is_lin && _dist_remaining.lin > 0)
            || (is_ang && _dist_remaining.ang > 0) )
    {
        // if we have to start breaking, set the desired velocity to the final one
        if (is_lin && !is_breaking_lin && should_be_breaking(
                    _dist_remaining.lin, _vel_current.lin, vel_final.lin, acc.lin))
        {
            vel_desired.lin = vel_final.lin;
            is_breaking_lin = true;

// TODO: for now do not do this, as it may cause a deadlock
//             acc.lin = required_breaking_acc(dist_remaining.lin, vel_current.lin, vel_final.lin);
// #if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
//             _acc.lin = acc.lin;
// #endif
        }
        if (is_ang && !is_breaking_ang && should_be_breaking(
                    _dist_remaining.ang, _vel_current.ang, vel_final.ang, acc.ang))
        {
            vel_desired.ang = vel_final.ang;
            is_breaking_ang = true;

// TODO: for now do not do this, as it may cause a deadlock
//             acc.ang = required_breaking_acc(_dist_remaining.ang, _vel_current.ang, vel_final.ang);
// #if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
//             _acc.ang = acc.ang;
// #endif
        }

#if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
            _vel_desired = vel_desired;
#endif

        // calculate new velocity
        float vel_new_lin = update_velocity(_vel_current.lin, vel_desired.lin, acc.lin);
        float vel_new_ang = update_velocity(_vel_current.ang, vel_desired.ang, acc.ang);

        // calculate the distance traveled using trapezoidal integration
        // (which should be "ideal" for constant accelerations)
        // trapezoidal rule: (a+b) / 2 * h
        float distance_traveled_lin = (_vel_current.lin + vel_new_lin) / 2 * _dt;
        float distance_traveled_ang = (_vel_current.ang + vel_new_ang) / 2 * _dt;

        // reduce remaining distance
        _dist_remaining.lin -= distance_traveled_lin;
        _dist_remaining.ang -= distance_traveled_ang;

        // update regulator target
        update_target_by(
                direction_lin * distance_traveled_lin,
                direction_ang * distance_traveled_ang);

        // save the new velocity as current
        _vel_current = {vel_new_lin, vel_new_ang};

        // wait
        delay(_dt);
    }

    // compensate for the last step if any distance is still remaining (t = s/v)
    // TODO: this may create errors if both velocities are non-zero
    float t_lin = (_vel_current.lin > 0) ? (_dist_remaining.lin / _vel_current.lin) : 0;
    float t_ang = (_vel_current.ang > 0) ? (_dist_remaining.ang / _vel_current.ang) : 0;
    delay(std::max(t_lin, t_ang));
    _dist_remaining.lin = _dist_remaining.ang = 0;

    // compensate for current velocity if we couldn't break fast enough
    // (we can, as we supply position, not velocity to the regulator)
    if (_vel_current.lin > vel_final.lin)
        _vel_current.lin = vel_final.lin;
    if (_vel_current.ang > vel_final.ang)
        _vel_current.ang = vel_final.ang;
}



bool should_be_breaking(float dist_remaining, float vel_current, float vel_final, float acc)  {
    float dist_required = (vel_current + vel_final) * (vel_current - vel_final) / (2 * acc);
    return dist_remaining <= dist_required;
}

// float required_breaking_acc(float dist_remaining, float vel_current, float vel_final)  {
//     float acc = (vel_current + vel_final) * (vel_current - vel_final) / (2 * dist_remaining);
//     return acc;
// }

float update_velocity(float vel_current, float vel_desired, float acc) {
    if (vel_current < vel_desired) {
        vel_current += acc * _dt;
        vel_current = std::min(vel_current, vel_desired); // remove overshoot
    } else if (vel_current > vel_desired) {
        vel_current -= acc * _dt;
        vel_current = std::max(vel_current, vel_desired); // remove overshoot
    }
    return vel_current;
}





} // namespace movement::controller
