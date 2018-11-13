#include "regulator.h"

#include "ubik/logging/logging.h"

namespace movement {
namespace regulator {

static constexpr int32_t MAX_ENCODER_READING = (1 << 12) - 1;
static constexpr float   LOOP_FREQUENCY = 1e3;


static void lock();
static void unlock();
static void update_position(int32_t &position, int32_t angle, int32_t last_angle);
static std::pair<float, float>
    convert_to_encoder_ticks(float translation_meters, float rotation_radians);

// SP and PV of PID regulation
// these are cumulative positions for each wheel
// we need to keep track of these as the angle wraps to zero after max value
// the unit is the same as for encoder readings
// for 12-bit resulution we can store ~53km of forward motion
static int32_t position_left = 0;
static int32_t position_right = 0;
// we need this to be float to avoid numerical errors
static float set_point_left = 0;
static float set_point_right = 0;

SemaphoreHandle_t state_mutex = nullptr;


void initialise() {
    static bool initialised = false;
    if (initialised) return;
    initialised = true;

    // create mutex to guard variables access (each will be short, but
    // must be guarded anyway)
    state_mutex = xSemaphoreCreateMutex();
    configASSERT(state_mutex != nullptr);
}

void regulation_task(void *) {
    // prepare PID regulators
    PID pid_left(LOOP_FREQUENCY), pid_right(LOOP_FREQUENCY);
    pid_left .set_params(3e-5, .1e-6/* , .1e-6 */);
    pid_right.set_params(3e-5, .1e-6/* , .1e-6 */);

    // get initial state (wheel angles)
    spi::EncoderReadings readings = spi::read_encoders();
    for (size_t i = 0; !readings.both_ok(); i++) {
        readings = spi::read_encoders();
        configASSERT(i < 10);
    }
    int32_t last_angle_left = readings.left.angle;
    int32_t last_angle_right = readings.right.angle;

    // prepare loop timing
    auto last_start = xTaskGetTickCount();
    size_t counter = 0;

    while (1) {
        // read encoders
        readings = spi::read_encoders();

        // accumulate wheel positions and save last angles
        if (readings.valid) {
            // left wheel readings decreases in forward direction, so negate it
            if (readings.left.is_ok()) {
                update_position(position_left, -readings.left.angle, -last_angle_left);
                last_angle_left = readings.left.angle;
            }
            // right wheel readings increase in forward direction
            if (readings.right.is_ok()) {
                update_position(position_right, readings.right.angle, last_angle_right);
                last_angle_right = readings.right.angle;
            }
        }

        // lock variables, do not wait!
        bool taken = xSemaphoreTake(state_mutex, 0) == pdPASS;

        if (taken) {
            // calculate PID output
            float out_left = pid_left.next(set_point_left, position_left);
            float out_right = pid_right.next(set_point_right, position_right);

            // normalize output so that we won't have to change parameters max_pulse changes
            // normalize such that PID output = 1.0 corresponds to 100% pulse
            out_left = out_left * motors::max_pulse();
            out_right = out_right * motors::max_pulse();

            // calculate motor directions and pulse
            motors::set_direction(
                    out_left  > 0 ? FORWARDS : BACKWARDS,
                    out_right > 0 ? FORWARDS : BACKWARDS);
            motors::set_pulse(
                    std::lround(std::abs(out_left)),
                    std::lround(std::abs(out_right)));


            // DEBUG
            if (counter++ % 50 == 0)
                logging::printf(120, "%8d,%8d,%8d,%8d,%8d,%8d\n",
                        (int) set_point_left, (int) position_left, (int) out_left,
                        (int) set_point_right, (int) position_right, (int) out_right);


            // release the mutex
            unlock();
        }

        // TODO: use a timer instead, for a higher frequency
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(1));
    }
}

void set_target(float translation_meters, float rotation_radians) {
    float ticks_left, ticks_right;
    std::tie(ticks_left, ticks_right)
        = convert_to_encoder_ticks(translation_meters, rotation_radians);

    lock();
    set_point_left = ticks_left;
    set_point_right = ticks_right;
    unlock();
}

void update_target_by(float translation_meters, float rotation_radians) {
    float ticks_left, ticks_right;
    std::tie(ticks_left, ticks_right)
        = convert_to_encoder_ticks(translation_meters, rotation_radians);

    lock();
    set_point_left += ticks_left;
    set_point_right += ticks_right;
    unlock();
}


void update_position(int32_t &position, int32_t angle, int32_t last_angle) {
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
std::pair<float, float> convert_to_encoder_ticks(float translation_meters, float rotation_radians)
{
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


static void lock() {
    // this should always succeed as we wait indefinitelly
    bool taken = xSemaphoreTake(state_mutex, portMAX_DELAY) == pdPASS;
    configASSERT(taken);
}

static void unlock() {
    // this could only fail if we didn't take the semaphore earlier
    bool could_give = xSemaphoreGive(state_mutex) == pdPASS;
    configASSERT(could_give);
}

} // namespace regulator
} // namespace movement
