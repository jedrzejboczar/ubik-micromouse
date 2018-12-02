#include "moves.h"

namespace movement {


Line::Line(float distance, float vel_desired, float acc, float vel_final):
    distance(distance), vel_desired(vel_desired), acc(acc), vel_final(vel_final) {}

void Line::initialise_generator(TrajectoryGenerator &g) {
    g.generate(distance, vel_desired, vel_final, acc);
}

Pair Line::convert_position(float generator_position_delta) {
    return std::make_pair(generator_position_delta, 0);
}

Rotate::Rotate(float angle, float vel_desired, float acc, float vel_final):
    angle(angle), vel_desired(vel_desired), acc(acc), vel_final(vel_final) {}

void Rotate::initialise_generator(TrajectoryGenerator &g) {
    g.generate(angle, vel_desired, vel_final, acc);
}

Pair Rotate::convert_position(float generator_position_delta) {
    return std::make_pair(0, generator_position_delta);
}

Arc::Arc(Pair distances, float vel_desired, float acc, float vel_final):
    distance(distances.first), vel_desired(vel_desired), acc(acc), vel_final(vel_final),
    curvature(distances.second / distances.first) {}

void Arc::initialise_generator(TrajectoryGenerator &g) {
    g.generate(distance, vel_desired, vel_final, acc);
}

Pair Arc::convert_position(float generator_position_delta) {
    float delta_lin = generator_position_delta;
    float delta_ang = delta_lin * curvature;
    return std::make_pair(delta_lin, delta_ang);
}


} // namespace movement
