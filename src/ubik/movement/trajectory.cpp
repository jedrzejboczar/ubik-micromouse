#include "trajectory.h"

namespace movement {


void TrajectoryGenerator::generate(float distance, float vel_desired, float vel_final, float acc) {
    this->s.dist_remaing = distance;
    this->s.vel_desired = vel_desired;
    this->s.vel_final = vel_final;
    this->s.acc = acc;
    // if the arguments are wrong, do not start
    if (distance <= 0 || vel_desired <= 0 || acc <= 0
            || vel_final < 0) // vel_final can be zero
        s.phase = FINISHED;
    else // 1st phase is acceleration, but it may actually instantly change later
        s.phase = ACCELERATING;
}

/*
 * This is a next step of the generation loop.
 * The basic algorithm (in loop form) is:
 *    initialise_movement_parameters
 *    while (distance > 0)
 *        if (should_be_breaking())
 *            vel_desired = vel_final
 *        update_velocity
 *        delta_position = integrate(velocity)
 *        distance -= delta_position
 *        yield_next_position_delta ->
 *        delay(dt)
 *    vel_current = vel_final
 */
float TrajectoryGenerator::next_position_delta(float dt) {
    // check if the movement is ongoing
    if (s.phase == FINISHED)
        return 0;

    // we will yield the distance traveled in this step
    float dist_traveled;

    // check if we should start breaking
    if (s.phase == ACCELERATING || s.phase == DESIRED_VELOCITY) {
        if (should_be_breaking(s.dist_remaing, s.vel_current, s.vel_final, s.acc)) {
            s.phase = DECELERATING;
            s.vel_desired = s.vel_final;
            // TODO: recalculate acceleration
            s.acc = required_breaking_acc(s.dist_remaing, s.vel_current, s.vel_final);
        }
    }

    // calculate the distance traveled in this step
    if (s.phase == ACCELERATING || s.phase == DECELERATING) {
        // integrate acceleration to get new velocity
        float vel_next = s.vel_current;
        if (s.vel_current < s.vel_desired) {
            vel_next += s.acc * dt;
            // remove overshoot
            if (vel_next >= s.vel_desired) {
                vel_next = s.vel_desired;
                s.phase = DESIRED_VELOCITY;
            }
        } else if (s.vel_current > s.vel_desired) {
            vel_next -= s.acc * dt;
            // remove overshoot
            if (vel_next <= s.vel_desired) {
                if (vel_next <= s.vel_final)
                    s.phase = FINISHED;
                else
                    s.phase = DESIRED_VELOCITY;
                vel_next = s.vel_desired;
            }
        }

        // use trapezoidal integration (trapezoidal rule: (a+b) / 2 * h)
        dist_traveled = (s.vel_current + vel_next) / 2 * dt;
        s.vel_current = vel_next;
    } else {
        // no need to change velocity, quadrature integration
        dist_traveled = s.vel_current * dt;
    }

    // update the remaining distance
    s.dist_remaing -= dist_traveled;

    // check the stop condition
    if (s.dist_remaing <= 0) {
        s.phase = FINISHED;
        // compensate for current velocity if we couldn't break fast enough
        // (we can, as we supply position, not velocity to the regulator)
        if (s.vel_current > s.vel_final)
            s.vel_current = s.vel_final;
    }

    return dist_traveled;
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
 */
bool TrajectoryGenerator::should_be_breaking(float dist_remaining, float vel_current, float vel_final, float acc) {
    float dist_required = (vel_current + vel_final) * (vel_current - vel_final) / (2 * acc);
    return dist_remaining <= dist_required;
}
/*
 * Now, for the required breaking acceleration it is just:
 *    a = (vc + vf)(vc - vf) / 2s
 */
float TrajectoryGenerator::required_breaking_acc(float dist_remaining, float vel_current, float vel_final)  {
    float acc = (vel_current + vel_final) * (vel_current - vel_final) / (2 * dist_remaining);
    return acc;
}

} // namespace movement
