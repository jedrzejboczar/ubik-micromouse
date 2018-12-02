#include "trajectory.h"
#include "trajectory.cpp"
#include "moves.h"
#include "moves.cpp"

#include <tuple>

#include <cmath>
#include <iostream>
// #include <iomanip>
// #include <vector>
#include <fstream>
// #include <cstdlib>
// #include <cassert>

typedef std::pair<float, float> Pair;

using namespace movement;

struct Position {
    float x, y, theta;
} pos = {0, 0, 0};

namespace regulator {
void update_target_by(float translation_meters, float rotation_radians) {
    pos.theta += rotation_radians;
    pos.x += translation_meters * std::cos(pos.theta);
    pos.y += translation_meters * std::sin(pos.theta);
}
}

template <typename T, int N>
constexpr int n_elements(T(&)[N]) {
    return N;
}

int main()
{
    constexpr float LOOP_FREQUENCY = 100;
    constexpr float dt = 1 / LOOP_FREQUENCY;
    float t = 0;

    constexpr float pi = 3.141592653589793;
    TrajectoryGenerator trajectory;

    // open our file
    std::ofstream file;
    file.open("sim_moves.csv");
    file << "t,x,y,theta" << std::endl;
    // log the data to file
    auto log = [&] () {
        file
            << t << ","
            << pos.x << ","
            << pos.y << ","
            << pos.theta
            << std::endl;
    };

    Line line1(1, 0.5, 0.2, 0);
    Rotate rot90(pi/2, pi/3, pi/4, 0);
    Arc arc({0.5, pi/2}, 0.3, 0.1, 0);
    float R = 1;
    Arc circle({2*pi*R, 2*pi}, 0.5, 0.2, 0);

    Move *moves[] = {&line1, &rot90, &line1, &rot90, &line1, &rot90, &line1,
        &arc, &circle};

    for (int i = 0; i < n_elements(moves); i++) {
        Move *move = moves[i];
        move->initialise_generator(trajectory);

        while (! trajectory.has_finished()) {
            log();

            // calculate next generator output
            float delta_pos = trajectory.next_position_delta(dt);

            // get the position updated according to the logic of this move
            Pair position_update = move->convert_position(delta_pos);

            // update regulator set-point
            std::apply(regulator::update_target_by, position_update);

            t += dt;
        }
        log();
    }

}



