#pragma once

/*
 * General class for generating movement trajectory x(t).
 *
 * We generate the trajectory (robot position in time) by constrolling
 * robot velocity. The velocity is somehow artificial, we can say that
 * the generator is "blind", i.e. it doesn't have any feedback, it just
 * yields subsequent positions that the robot should track.
 *
 * The velocity is changed according to the motion parameters:
 *  - acceleration
 *  - desired velocity (max velocity)
 *  - initial velocity (velocity at the begginging of the move)
 *  - final velocity (velocity at the end of the move)
 * The movement takes as much time as is required to reach the given
 * distance.
 *
 * For the case with constant acceleration (only const acc is supported)
 * the idea is as follows:
 *   adjust `vel_current` to obtain desired trapezoidal shape,
 *   where `acc` gives the speed of change:
 *
 *     ^
 *     |        vel_desired__ __________
 * vel |                     /          \
 *     |                    /            \_____ __vel_final
 *     | vel_initial__ ____/
 *     +---------------------------------------------------->
 *                              time
 *
 * This corresponds to phases:
 *    ACCELERATING, DESIRED_VELOCITY, DECELERATING, FINISHED
 *
 * It may be the case that `vel_desired` cannot be reached, because the
 * acceleration is to low. It may also be the case that we do not even
 * start accelerating, when `vel_initial` is higher than `vel_desired`
 * and we have to decelerate for the whole movement.
 * It is also possible that we start with DECELERATING, then we reach
 * DESIRED_VELOCITY, and then again we are in DECELERATING phase.
 *
 * The acceleration is adjusted when we start decelerating in such
 * a way that we always reach vel_final at the end of the movement.
 * In practice this means that the deceleration is slightly higher
 * than acceleration (this depends on how often we sample).
 * IMPORTANT: This also means that, when we have to decelerate for the
 * whole movement, the `acc` will not be as requested. When the movement
 * requires very fast deceleration (e.g. we specify that we want to
 * reach zero velocity on a short distance, when the current velocity
 * is high), the actual deceleration used will be MUCH higher than `acc`.
 * Anyway, it is probably more imporatant to decelerate fast on the
 * given distance and possibly loose track of position (due to slip),
 * than to decelerate gently and e.g. hit the wall.
 */

namespace movement {

class TrajectoryGenerator {
public:
    enum MovementPhase {
        ACCELERATING, DESIRED_VELOCITY, DECELERATING, FINISHED
    };

    // used to avoid 6 const getters and have just one
    struct State {
        MovementPhase phase;
        float dist_remaing;
        float vel_current;
        float vel_desired;
        float vel_final;
        float acc;
        // all others should be set by generate()
        State(float vel_initial): phase(FINISHED), vel_current(vel_initial) {}
    };

    TrajectoryGenerator(float vel_initial = 0): s(vel_initial) {}

    const State& state() const { return s; }
    bool has_finished() const { return s.phase == FINISHED; }

    // all the arguments have to be positive
    void generate(float distance, float vel_desired, float vel_final, float acc);
    float next_position_delta(float dt);
private:
    // store current state variables
    State s;

    // pure functions
    static bool should_be_breaking(float dist_remaining, float vel_current, float vel_final, float acc);
    static float required_breaking_acc(float dist_remaining, float vel_current, float vel_final);

    // for testing to get access to private members
    friend class TrajectoryGenerator_TEST;
};

} // namespace movement
