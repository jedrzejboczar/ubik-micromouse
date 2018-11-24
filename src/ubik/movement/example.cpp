#include <cstdio>

#include "controller.h"
#include "controller.cpp"

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <cassert>

using movement::Vec2;

std::ostream& operator<<(std::ostream& os, const Vec2& obj)
{
    os << obj.lin << "," << obj.ang;
    return os;
}

/*** Gather data for plotting by implementing controller methods **************/

struct {
    float t = 0;
    std::vector<float> time;
    // controller internal data
    std::vector<Vec2> dist_remaining;
    std::vector<Vec2> vel_current;
    std::vector<Vec2> vel_desired;
    std::vector<Vec2> acc;
    // simulate robot movement based on regulator set-point
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> theta;
    float x_curr = 0, y_curr = 0, theta_curr = 0;
} data;

void movement::update_target_by(float distance_linear, float distance_angular) {
    data.theta_curr += distance_angular;
    data.x_curr += std::cos(data.theta_curr) * distance_linear;
    data.y_curr += std::sin(data.theta_curr) * distance_linear;
    data.x.push_back(data.x_curr);
    data.y.push_back(data.y_curr);
    data.theta.push_back(data.theta_curr);
}

void movement::Controller::delay(float dt) {
    data.time.push_back(data.t);
    data.dist_remaining.push_back(dist_remaining);
    data.vel_current.push_back(vel_current);
#if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
    data.vel_desired.push_back(vel_desired);
    data.acc.push_back(acc);
#else
    data.vel_desired.push_back({0, 0});
    data.acc.push_back({0, 0});
#endif

    data.t += dt;

    assert(data.time.size() < 1000000);

#if 0
    std::cout << "Current values:" << std::endl
        << std::fixed << std::setprecision(6)
        << "  dist_remaining = " << dist_remaining << std::endl
        << "  vel_current    = " << vel_current << std::endl
#if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
        << "  vel_desired    = " << vel_desired << std::endl
        << "  acc            = " << acc << std::endl
#endif
        ;
#endif

    // int ms = 1000 * dt;
    // std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void save_to_file() {
    std::ofstream file;
    file.open("sim_lin.csv");
    file << "time,dist_remaining,vel_current,vel_desired,acc" << std::endl;
    for (size_t i = 0; i < data.time.size(); i++) {
        file
            << data.time[i] << ","
            << data.dist_remaining[i].lin << ","
            << data.vel_current[i].lin << ","
#if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
            << data.vel_desired[i].lin << ","
            << data.acc[i].lin
#else
            << "NaN" << ","
            << "NaN"
#endif
            << std::endl;
    }
    file.close();

    file.open("sim_ang.csv");
    file << "time,dist_remaining,vel_current,vel_desired,acc" << std::endl;
    for (size_t i = 0; i < data.time.size(); i++) {
        file
            << data.time[i] << ","
            << data.dist_remaining[i].ang << ","
            << data.vel_current[i].ang << ","
#if DEBUG_STORE_VEL_DESIRED_AND_ACC == 1
            << data.vel_desired[i].ang << ","
            << data.acc[i].ang
#else
            << "NaN" << ","
            << "NaN"
#endif
            << std::endl;
    }
    file.close();

    file.open("sim_pos.csv");
    file << "time,x,y,theta" << std::endl;
    for (size_t i = 0; i < data.x.size(); i++) {
        file
            << data.time[i] << ","
            << data.x[i] << ","
            << data.y[i] << ","
            << data.theta[i] << std::endl;
    }
    file.close();
}

int main()
{
    int counter = 0;
    auto cnt = [&counter] () {
        std::cout << counter++ << ". size: " << data.time.size() << ", time: " << data.t << std::endl;
    };

    float f = 1000;
    movement::Controller c(f);
    // move_line(distance, vel_desired, acc, vel_final=0)
    float acc = 1.200;

    c.move_line(0.5, 1.0, acc);
    c.move_rotate(2*PI/3, 1.0, acc);
    c.move_line(0.5, 1.0, acc);
    c.move_rotate(2*PI/3, 1.0, acc);
    c.move_line(0.5, 1.0, acc);
    c.move_rotate(5*PI/6, 1.0, acc);

    c.move_arc({0.8, PI}, 1.3, acc, 0.9); cnt();
    c.move_arc({1.3, PI}, 0.9, acc); cnt();
    c.move_arc({0.8, -PI}, 1.3, acc); cnt();
    c.move_arc({-0.9, PI}, 1.3, acc); cnt();
    c.move_arc({-0.3, -PI}, 1.3, acc); cnt();

    save_to_file();
    system("python example.py");
}
