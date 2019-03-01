#include "moves.h"

#include <cmath>

namespace movement {

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Line::Line(float distance, float vel_desired, float acc, float vel_final):
    distance(distance), vel_desired(vel_desired), acc(acc), vel_final(vel_final) {}

void Line::initialise_generator(TrajectoryGenerator &g) {
    g.generate(fabsf(distance), vel_desired, vel_final, acc);
}

Pair Line::convert_position(float generator_position_delta) {
    return std::make_pair(sgn(distance) * generator_position_delta, 0);
}

Rotate::Rotate(float angle, float vel_desired, float acc, float vel_final):
    angle(angle), vel_desired(vel_desired), acc(acc), vel_final(vel_final) {}

void Rotate::initialise_generator(TrajectoryGenerator &g) {
    g.generate(fabsf(angle), vel_desired, vel_final, acc);
}

Pair Rotate::convert_position(float generator_position_delta) {
    return std::make_pair(0, sgn(angle) * generator_position_delta);
}

Arc::Arc(Pair distances, float vel_desired, float acc, float vel_final):
    distance(distances.first), vel_desired(vel_desired), acc(acc), vel_final(vel_final),
    curvature(distances.second / distances.first) {}

void Arc::initialise_generator(TrajectoryGenerator &g) {
    g.generate(fabsf(distance), vel_desired, vel_final, acc);
}

Pair Arc::convert_position(float generator_position_delta) {
    float delta_lin = generator_position_delta;
    float delta_ang = delta_lin * curvature;
    // for the angular we have to unwind the real sign of distances.second
    return std::make_pair(sgn(distance) * delta_lin,
            sgn(distance) * sgn(curvature) * delta_ang);
}


} // namespace movement
