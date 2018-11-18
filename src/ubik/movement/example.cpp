#include "controller.h"

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <cassert>

float t = 0;
using movement::Vec2;
std::vector<float> g_time;
std::vector<Vec2> g_dist_remaining;
std::vector<Vec2> g_vel_current;
std::vector<Vec2> g_vel_desired;
std::vector<Vec2> g_acc;

std::ostream& operator<<(std::ostream& os, const Vec2& obj)
{
    os << obj.lin << "," << obj.ang;
    return os;
}

void movement::Controller::delay(float dt) {
    g_time.push_back(t);
    g_dist_remaining.push_back(dist_remaining);
    g_vel_current.push_back(vel_current);
    g_vel_desired.push_back(vel_desired);
    g_acc.push_back(acc);

    t += dt;

    assert(g_time.size() < 1000000);

    // std::cout << "Current values:" << std::endl
    //     << std::fixed << std::setprecision(3)
    //     << "  dist_remaining = " << dist_remaining << std::endl
    //     << "  vel_current = " << vel_current << std::endl
    //     << "  vel_desired = " << vel_desired << std::endl
    //     << "  acc = " << acc << std::endl;

    // int ms = 1000 * dt;
    // std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void save_to_file() {
    std::ofstream file;
    file.open("sim_lin.csv");
    file << "time,dist_remaining,vel_current,vel_desired,acc" << std::endl;
    for (size_t i = 0; i < g_time.size(); i++) {
        file
            << g_time[i] << ","
            << g_dist_remaining[i].lin << ","
            << g_vel_current[i].lin << ","
            << g_vel_desired[i].lin << ","
            << g_acc[i].lin << std::endl;
    }
    file.close();
    file.open("sim_ang.csv");
    file << "time,dist_remaining,vel_current,vel_desired,acc" << std::endl;
    for (size_t i = 0; i < g_time.size(); i++) {
        file
            << g_time[i] << ","
            << g_dist_remaining[i].ang << ","
            << g_vel_current[i].ang << ","
            << g_vel_desired[i].ang << ","
            << g_acc[i].ang << std::endl;
    }
    file.close();
}

int main()
{
    int counter = 0;
    auto cnt = [&counter] () { std::cout << counter++ << ". " << g_time.size() << std::endl; };

    float f = 1000;
    movement::Controller c(f);
    // move_line(distance, vel_desired, acc, vel_final=0)
    float acc = 2.5;
    c.move_arc({0.8, 0}, {1.3, 0}, {acc, 0.0}, {0.8, 0}); cnt();
    c.move_arc({1.0, 0}, {1.0, 0}, {acc, acc}); cnt();
    c.move_arc({0, 1.0}, {0, 0.6}, {acc, acc}); cnt();
    c.move_arc({1.0, 0}, {0.5, 0}, {acc, 0.0}); cnt();
    c.move_arc({1., 1.}, {.3, .4}, {acc, acc}); cnt();
    c.move_arc({1.0, 0}, {0.5, 0}, {acc, acc}); cnt();

    save_to_file();
    system("python example.py");
}
