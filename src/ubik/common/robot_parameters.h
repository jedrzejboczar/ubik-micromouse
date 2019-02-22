#pragma once

namespace constants {

// taken from arm_math.h, as define to avoid problems when someone already defines it
#ifndef PI
#define PI                 3.14159265358979f
#endif

static constexpr float RIGHT_ANGLE = PI/2;

static constexpr float WHEEL_RADIUS        = 0.01600;             // 16.26 mm
static constexpr float WHEEL_CIRCUMFERENCE = 2*PI * WHEEL_RADIUS; // 0.1022 m
static constexpr float TURN_RADIUS         = 0.0475;              // 49.3 mm
static constexpr float TURN_CIRCUMFERENCE  = 2*PI * TURN_RADIUS;
static constexpr int   GEAR_RATIO          = 30;                  // 30 shaft truns per wheel turn


static inline float deg2rad(float deg) {
    return deg / 180 * PI;
}

constexpr float c_deg2rad(float deg) {
    return deg / 180 * PI;
}

static inline float rad2deg(float rad) {
    return rad / PI * 180;
}

static inline float arc_length(float angle, float radius) {
    return angle * radius;
}


static inline float rotation_angle2arc_length(float angle) {
    return arc_length(angle, TURN_RADIUS);
}
static inline float arc_length2rotation_angle(float arc_length) {
    return arc_length / TURN_RADIUS;
}


} // namespace constants
