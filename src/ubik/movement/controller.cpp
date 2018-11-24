#include "controller.h"

namespace movement {


void Controller::move_arc(Vec2 distance, Vec2 vel_desired, Vec2 acc, Vec2 vel_final) {
    int direction_lin = distance.lin > 0 ? 1 : -1;
    int direction_ang = distance.ang > 0 ? 1 : -1;

    // use only absolute values, direction only affects the resulting velocities sign
    distance = abs(distance);
    vel_desired = abs(vel_desired);
    acc = abs(acc);
    vel_final = abs(vel_final);

#if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
    this->vel_desired = vel_desired;
    this->acc = acc;
#endif

    // save the remaining distance to a member variable
    dist_remaining = distance;

    // determine if one of the movements is not being used
    // this is needed for breaking calculations to avoid division by zero
    bool is_lin = distance.lin > 0 && vel_desired.lin > 0 && acc.lin > 0;
    bool is_ang = distance.ang > 0 && vel_desired.ang > 0 && acc.ang > 0;

    // keep track if breaking has been started
    bool is_breaking_lin = false;
    bool is_breaking_ang = false;

    // while ( (is_lin && dist_remaining.lin > vel_current.lin * dt)
    //         || (is_ang && dist_remaining.ang > vel_current.ang * dt) )
    while ( (is_lin && dist_remaining.lin > 0)
            || (is_ang && dist_remaining.ang > 0) )
    {
        // if we have to start breaking, set the desired velocity to the final one
        if (is_lin && !is_breaking_lin && should_be_breaking(
                    dist_remaining.lin, vel_current.lin, vel_final.lin, acc.lin))
        {
            vel_desired.lin = vel_final.lin;
            is_breaking_lin = true;

// TODO: for now do not do this, as it may cause a deadlock
//             acc.lin = required_breaking_acc(dist_remaining.lin, vel_current.lin, vel_final.lin);
// #if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
//             this->acc.lin = acc.lin;
// #endif
        }
        if (is_ang && !is_breaking_ang && should_be_breaking(
                    dist_remaining.ang, vel_current.ang, vel_final.ang, acc.ang))
        {
            vel_desired.ang = vel_final.ang;
            is_breaking_ang = true;

// TODO: for now do not do this, as it may cause a deadlock
//             acc.ang = required_breaking_acc(dist_remaining.ang, vel_current.ang, vel_final.ang);
// #if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
//             this->acc.ang = acc.ang;
// #endif
        }

#if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
            this->vel_desired = vel_desired;
#endif

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

        // update regulator target
        update_target_by(
                direction_lin * distance_traveled_lin,
                direction_ang * distance_traveled_ang);

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

    // compensate for current velocity if we couldn't break fast enough
    // (we can, as we supply position, not velocity to the regulator)
    if (vel_current.lin > vel_final.lin)
        vel_current.lin = vel_final.lin;
    if (vel_current.ang > vel_final.ang)
        vel_current.ang = vel_final.ang;
}

bool Controller::should_be_breaking(float dist_remaining, float vel_current, float vel_final, float acc)  {
    float dist_required = (vel_current + vel_final) * (vel_current - vel_final) / (2 * acc);
    return dist_remaining <= dist_required;
}

float Controller::required_breaking_acc(float dist_remaining, float vel_current, float vel_final)  {
    float acc = (vel_current + vel_final) * (vel_current - vel_final) / (2 * dist_remaining);
    return acc;
}

float Controller::update_velocity(float vel_current, float vel_desired, float acc) {
    if (vel_current < vel_desired) {
        vel_current += acc * dt;
        vel_current = std::min(vel_current, vel_desired); // remove overshoot
    } else if (vel_current > vel_desired) {
        vel_current -= acc * dt;
        vel_current = std::max(vel_current, vel_desired); // remove overshoot
    }
    return vel_current;
}





} // namespace movement
