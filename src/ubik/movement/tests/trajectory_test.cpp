#include "trajectory.h"
#include "trajectory.cpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <cassert>

namespace movement {

class TrajectoryGenerator_TEST {
public:

    // BRUTAL test
    static void test_for_infinite_loops() {
        constexpr int MAX_N_LOOPS = 100000;
        for (float dist = 0.0; dist < 1; dist+=0.1) {
            for (float vel_d = 0.0; vel_d < 1; vel_d+=0.1) {
                for (float vel_f = 0; vel_f < 1; vel_f+=0.1) {
                    for (float acc = 0.0; acc < 1; acc+=0.1) {
                        for (float dt: {0.001, 0.005, 0.01, 0.05}) {
                            movement::TrajectoryGenerator gen;
                            gen.generate(dist, vel_d, vel_f, acc);
                            int i;
                            for (i = 0; gen.phase != movement::TrajectoryGenerator::FINISHED && i < MAX_N_LOOPS; i++) {
                                gen.next_position_delta(dt);
                            }

                            char params_str[200] = {0};
                            std::sprintf(params_str, "d=%6.3f, vd=%6.3f, vf=%6.3f, a=%6.3f, dt=%6.3f",
                                    dist, vel_d, vel_f, acc, dt);

                            bool anything_is_zero = dist <= 0 || vel_d <= 0 || acc <= 0;

                            if (gen.phase != movement::TrajectoryGenerator::FINISHED || (!anything_is_zero && i <= 1)) {
                                char state_str[200] = {0};
                                std::sprintf(state_str, "dr=%6.3f, vc=%6.3f, vd=%6.3f, p=%1d",
                                        gen.dist_remaing, gen.vel_current, gen.vel_desired, gen.phase);
                                std::fprintf(stderr,
                                        "[ERR] i = %6d for [%s] state [%s]\n",
                                        i, params_str, state_str
                                        );
                            }
                            else if (0)
                                std::fprintf(stderr,
                                        "[ok]  i = %6d for [%s]\n",
                                        i, params_str
                                        );
                        }

                    }
                }
            }
        }
    }

    static void main() {
        movement::TrajectoryGenerator gen;

        constexpr int MAX_N_LOOPS = 100000;
        float dt = 0.01, t = 0;

        // open our file
        std::ofstream file;
        file.open("sim.csv");
        file << "time,dist_remaining,vel_current,vel_desired,vel_final,acc,phase" << std::endl;
        // log the data to file
        auto log = [&] () {
            file
                << t << ","
                << gen.dist_remaing << ","
                << gen.vel_current << ","
                << gen.vel_desired << ","
                << gen.vel_final << ","
                << gen.acc << ","
                << gen.phase
                << std::endl;
        };

        // perform one test
        auto loop_until_finished = [&] () {
            int i;
            for (i = 0; gen.phase != TrajectoryGenerator::FINISHED && i < MAX_N_LOOPS; i++) {
                log();
                float d_pos = gen.next_position_delta(dt);
                (void) d_pos;
                t += dt; // delay
            }
            log();
            bool success = gen.phase == TrajectoryGenerator::FINISHED && i > 1;
            if (!success)
                std::cerr << "Loop not finished correctly!!! (i = " << i << ")" << std::endl;
        };

        gen.generate(0.5, 0.3, 0, 0.4); loop_until_finished();
        gen.generate(0.5, 0.5, 0, 0.1); loop_until_finished();
        gen.generate(0.5, 0.3, 0.3, 0.2); loop_until_finished();
        gen.generate(0.5, 0.2, 0, 0.1); loop_until_finished();


        log();

        test_for_infinite_loops();

    }
};


} // namespace movement


int main()
{
    movement::TrajectoryGenerator_TEST::main();
}

