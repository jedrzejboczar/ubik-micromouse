#include "odometry.h"

#include <arm_math.h>

namespace localization {

// guard access to the internal variables of the module
static SemaphoreHandle_t internal_mutex = nullptr;

/*
 * Store total cumulative position of each wheel.
 * We need to keep track of these, as the encoders angle wraps to zero after
 * the max value is reached.
 * The units are encoder ticks (directly from encoders).
 * About overflows: for 12-bit resulution we can store ~53km of forward motion.
 */
static int32_t cumulative_position_left = 0;
static int32_t cumulative_position_right = 0;
static int32_t last_angle_left;
static int32_t last_angle_right;


static Position current_position = {0, 0, 0};


static void lock();
static void unlock();
static int32_t unwrap_encoder_angle_delta(int32_t angle_delta);
static void update_global_position(int32_t left_delta, int32_t right_delta);


void initialise() {
    static bool initialised = false;
    if (initialised) return;
    initialised = true;

    internal_mutex = xSemaphoreCreateMutex();
    configASSERT(internal_mutex != nullptr);

    bool encoders_ok = reset_wheels_state();
    configASSERT(encoders_ok);
}


bool reset_wheels_state() {
    // get initial state of the wheel angles
    spi::EncoderReadings readings = spi::read_encoders();
    // try few more times
    const int max_attempts = 5;
    for (int i = 0; !readings.both_ok(); i++) {
        if (i >= max_attempts) return false;
        readings = spi::read_encoders();
    }

    lock();
    last_angle_left = readings.left.angle;
    last_angle_right = readings.right.angle;
    unlock();

    return true;
}


bool next_encoders_reading() {
    // read the encoders
    spi::EncoderReadings readings = spi::read_encoders();

    if (! readings.valid)
        return false;

    // need to update positions
    lock();

    // if any angle is wrong, we have to assume that it didn't change
    if (! readings.left.is_ok())
        readings.left.angle = last_angle_left;
    if (! readings.right.is_ok())
        readings.right.angle = last_angle_right;

    // left wheel readings decreases in forward direction, so negate it
    // right wheel readings increase in forward direction
    int32_t left_delta = (-readings.left.angle) - (-last_angle_left);
    int32_t right_delta = readings.right.angle - last_angle_right;

    left_delta = unwrap_encoder_angle_delta(left_delta);
    right_delta = unwrap_encoder_angle_delta(right_delta);

    // accumulate wheel positions and save last angles
    cumulative_position_left += left_delta;
    cumulative_position_right += right_delta;

    // accumulate the global position
    update_global_position(left_delta, right_delta);

    last_angle_left = readings.left.angle;
    last_angle_right = readings.right.angle;

    unlock();

    return true;
}


std::pair<int32_t, int32_t> get_cumulative_encoder_ticks() {
    lock();
    auto ticks = std::make_pair(cumulative_position_left, cumulative_position_right);
    unlock();
    return ticks;
}

Position get_current_position() {
    lock();
    auto pos = current_position;
    unlock();
    return pos;
}

void set_current_position(Position pos) {
    lock();
    current_position = pos;
    unlock();
}

/*
 * Convert from translation/rotation to the total angle of the wheels.
 *    v_lin = (v_right + v_left) / 2
 *    v_ang = (v_right - v_left) / 2
 * so:
 *    v_left = v_lin - v_ang
 *    v_right = v_lin + v_ang
 * and this (for velocity) also translates to distance.
 * About the direction:
 *    When v_ang is positive, we turn left, looking from top.
 *    This is the same as with positive angle in Euclidean coordinates system.
 */
std::pair<float, float> convert_to_encoder_ticks(float translation_meters, float rotation_radians) {
    // convert trans/rot to distance traveled by each wheel
    float distance_left_m = translation_meters - constants::rotation_angle2arc_length(rotation_radians);
    float distance_right_m = translation_meters + constants::rotation_angle2arc_length(rotation_radians);
    // convert distance of the wheel to number of encoder turns
    // (encoders are after gear transmission, so they turn much faster)
    float n_turns_left = distance_left_m / constants::WHEEL_CIRCUMFERENCE * constants::GEAR_RATIO;
    float n_turns_right = distance_right_m / constants::WHEEL_CIRCUMFERENCE * constants::GEAR_RATIO;
    // convert number of turns to encoder ticks
    float n_ticks_left = n_turns_left * MAX_ENCODER_READING;
    float n_ticks_right = n_turns_right * MAX_ENCODER_READING;
    return std::make_pair(n_ticks_left, n_ticks_right);
}

std::pair<float, float> convert_to_translation_rotation(float left_angle, float right_angle) {
    // to number of turns
    float n_turns_left = left_angle / MAX_ENCODER_READING / constants::GEAR_RATIO;
    float n_turns_right = right_angle / MAX_ENCODER_READING / constants::GEAR_RATIO;
    // to linear distnace for the wheel
    float distance_left_m = n_turns_left * constants::WHEEL_CIRCUMFERENCE;
    float distance_right_m = n_turns_right * constants::WHEEL_CIRCUMFERENCE;
    // to transaltion/rotation
    float translation_m = (distance_right_m + distance_left_m) / 2;
    float rotation_rad = constants::arc_length2rotation_angle( (distance_right_m - distance_left_m) / 2);
    return std::make_pair(translation_m, rotation_rad);
}

static int32_t unwrap_encoder_angle_delta(int32_t angle_delta) {
    // detect the right direction of movement
    // if delta is greater than half turn, we assume it is movement in other direction
    if (std::abs(angle_delta) > MAX_ENCODER_READING / 2) {
        if (angle_delta >= 0)
            angle_delta -= MAX_ENCODER_READING;
        else
            angle_delta += MAX_ENCODER_READING;
    }
    return angle_delta;
}

static void update_global_position(int32_t left_delta, int32_t right_delta) {
    float translation_m, rotation_rad;
    std::tie(translation_m, rotation_rad) = convert_to_translation_rotation(left_delta, right_delta);
    float theta = current_position.theta;
    float delta_x = translation_m * arm_cos_f32(theta); // theta must be in range [0, 2pi]!
    float delta_y = translation_m * arm_sin_f32(theta); // theta must be in range [0, 2pi]!
    // TODO: trapezoidal integration
    current_position.x += delta_x;
    current_position.y += delta_y;
    current_position.theta += rotation_rad;
    // wrap angle to [0, 2pi]
    if (current_position.theta < 0)
        current_position.theta += 2*PI;
    else if (current_position.theta >= 2*PI)
        current_position.theta -= 2*PI;
}

static void lock() {
    // this should always succeed as we wait indefinitelly
    bool taken = xSemaphoreTake(internal_mutex, portMAX_DELAY) == pdPASS;
    configASSERT(taken);
}

static void unlock() {
    // this could only fail if we didn't take the semaphore earlier
    bool could_give = xSemaphoreGive(internal_mutex) == pdPASS;
    configASSERT(could_give);
}


} // namespace localization
