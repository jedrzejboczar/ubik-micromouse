#pragma once

#include <tuple>

#include "trajectory.h"

namespace movement {

typedef std::pair<float, float> Pair;


class Move {
public:
    virtual ~Move() {}
    virtual void initialise_generator(TrajectoryGenerator &generator) = 0;
    virtual Pair convert_position(float generator_position_delta) = 0;
};


class Line: public Move {
    float distance, vel_desired, acc, vel_final;
public:
    Line(float distance, float vel_desired, float acc, float vel_final);
    virtual void initialise_generator(TrajectoryGenerator &generator) override;
    virtual Pair convert_position(float generator_position_delta) override;
};


class Rotate: public Move {
    float angle, vel_desired, acc, vel_final;
public:
    Rotate(float angle, float vel_desired, float acc, float vel_final);
    virtual void initialise_generator(TrajectoryGenerator &generator) override;
    virtual Pair convert_position(float generator_position_delta) override;
};

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
 */
class Arc: public Move {
    float distance, vel_desired, acc, vel_final;
    float curvature; // k = angle / distance_lin
public:
    Arc(Pair distances, float vel_desired, float acc, float vel_final);
    virtual void initialise_generator(TrajectoryGenerator &generator) override;
    virtual Pair convert_position(float generator_position_delta) override;
};

/*
 * TODO: Euler spiral
 * For more gentle turns we should use a combination of euler spirals and arcs.
 * Euler spiral should probably be modeled as Arc with non-constant curvature,
 * probably depending on the ratio vel_current/vel_desired?
 * The important part is to calculate the movement parameters so that we can
 * specify the whole path with just two parameters (as in Arc). This should
 * be calculated in the constructor if possible.
 */

} // namespace movement
