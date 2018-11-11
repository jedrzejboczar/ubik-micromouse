#include "controller.cpp"

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
std::vector<float> g_time;
std::vector<float> g_dist_remaining;
std::vector<float> g_vel_current;
std::vector<float> g_vel_desired;
std::vector<float> g_acc;

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
    file.open("sim.csv");
    file << "time,dist_remaining,vel_current,vel_desired,acc" << std::endl;
    for (size_t i = 0; i < g_time.size(); i++) {
        file
            << g_time[i] << ","
            << g_dist_remaining[i] << ","
            << g_vel_current[i] << ","
            << g_vel_desired[i] << ","
            << g_acc[i] << std::endl;
    }
    file.close();
}

int main()
{
    float dt = 0.001;
    movement::Controller c(dt);
    // move_line(distance, vel_desired, acc, vel_final=0)
    float acc = 2.0;
    c.move_line(3.0, 2, acc, 1);
    c.move_line(2.0, 1, acc, 0.5);
    c.move_line(3.0, 0.5, acc);

    save_to_file();
    system("python example.py");
}
