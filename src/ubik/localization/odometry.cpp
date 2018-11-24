#include "odometry.h"

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


static void lock();
static void unlock();
static void update_position(int32_t &position, int32_t angle, int32_t last_angle);


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

    // accumulate wheel positions and save last angles
    // left wheel readings decreases in forward direction, so negate it
    if (readings.left.is_ok()) {
        update_position(cumulative_position_left, -readings.left.angle, -last_angle_left);
        last_angle_left = readings.left.angle;
    }
    // right wheel readings increase in forward direction
    if (readings.right.is_ok()) {
        update_position(cumulative_position_right, readings.right.angle, last_angle_right);
        last_angle_right = readings.right.angle;
    }

    unlock();

    return true;
}


std::pair<int32_t, int32_t> get_cumulative_encoder_ticks() {
    lock();
    auto ticks = std::make_pair(cumulative_position_left, cumulative_position_right);
    unlock();
    return ticks;
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


static void update_position(int32_t &position, int32_t angle, int32_t last_angle) {
    int32_t angle_delta = angle - last_angle;
    // detect the right direction of movement
    // if delta is greater than half turn, we assume it is movement in other direction
    if (std::abs(angle_delta) > MAX_ENCODER_READING / 2) {
        if (angle_delta >= 0)
            angle_delta -= MAX_ENCODER_READING;
        else
            angle_delta += MAX_ENCODER_READING;
    }
    // integrate the position delta to obtain cumulative position
    position += angle_delta;
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
