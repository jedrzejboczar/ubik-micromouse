#include "regulator.h"

#include "ubik/logging/logging.h"
#include "ubik/common/timing.h"

#define TIMING_ENABLED 0


namespace movement {

// this is implemented to interface with controller.h
void update_target_by(float distance_linear, float distance_angular) {
    regulator::update_target_by(distance_linear, distance_angular);
}


namespace regulator {

// SetPoint of PID regulation
// keep the value is in encoder ticks for more direct relation to encoder
// readings; we need these values to be float to avoid numerical errors
static float set_point_left = 0;
static float set_point_right = 0;

// internal state of regulator: enabling/disabling
static bool regulation_enabled = false;

// needed to guard all global variables access from different modules
// (all public functions should use the mutex)
SemaphoreHandle_t state_mutex = nullptr;

// timing stats
float mean_regulation_time_us = 0;
int32_t mean_regulation_time_i = 0;


static void lock();
static void unlock();


void initialise() {
    static bool initialised = false;
    if (initialised) return;
    initialised = true;

    // create mutex to guard variables access (each will be short, but
    // must be guarded anyway)
    state_mutex = xSemaphoreCreateMutex();
    configASSERT(state_mutex != nullptr);
}

void set_enabled(bool enabled) {
    // if we enable regulation we have to reset regulation state
    lock();
    regulation_enabled = enabled;
    unlock();
    motors::set_enabled(enabled);
}

void regulation_task(void *) {
    // prepare PID regulators
    PID pid_left, pid_right;
    pid_left .set_params(3e-5, .1e-6/* , .1e-6 */);
    pid_right.set_params(3e-5, .1e-6/* , .1e-6 */);

    // avoid jumps of PV
    localization::reset_wheels_state();

    // prepare loop timing
    auto last_start = xTaskGetTickCount();
    bool is_enabled = regulation_enabled;

    // go into regulation loop
    while (1) {

#if TIMING_ENABLED == 1
        // timing start
        cycles_counter::reset();
        cycles_counter::start();
#endif

        // perform next encoders reading and get the current process variable values
        localization::next_encoders_reading();
        int32_t position_left, position_right;
        std::tie(position_left, position_right) = localization::get_cumulative_encoder_ticks();

        // lock variables, do not wait! (as regulation may be faster than single rtos tick)
        bool taken = xSemaphoreTake(state_mutex, 0) == pdPASS;

        if (taken) {
            // check if there was a request to enable/disable the regulator
            if (is_enabled != regulation_enabled) {
                is_enabled = regulation_enabled;
                // we need to reset state on re-enabling
                if (is_enabled) {
                    set_point_left = position_left;
                    set_point_right = position_right;
                    pid_left.reset_state();
                    pid_right.reset_state();
                }
            }

            if (is_enabled) {
                // calculate PID output
                float dt = 1.0f / LOOP_FREQUENCY;
                float out_left = pid_left.next(set_point_left, position_left, dt);
                float out_right = pid_right.next(set_point_right, position_right, dt);

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
            }

            // // DEBUG
            // static size_t counter = 0;
            // if (counter++ % 10 == 0)
            //     logging::printf(120, "%8d,%8d,%8d,%8d,%8d,%8d\n",
            //             (int) set_point_left, (int) position_left, (int) out_left,
            //             (int) set_point_right, (int) position_right, (int) out_right);


            // release the mutex
            unlock();
        }

#if TIMING_ENABLED == 1
        // end timing
        cycles_counter::stop();
        uint32_t micros = cycles_counter::get_us();
        // m[n] = m[n-1] + (a[n] - m[n-1]) / n
        mean_regulation_time_us += (micros - mean_regulation_time_us) / (mean_regulation_time_i++ + 1);
#endif

        // TODO: use a timer instead, for a higher frequency
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(1000 / LOOP_FREQUENCY));
    }
}

void set_target(float translation_meters, float rotation_radians) {
    float ticks_left, ticks_right;
    std::tie(ticks_left, ticks_right)
        = localization::convert_to_encoder_ticks(translation_meters, rotation_radians);

    lock();
    set_point_left = ticks_left;
    set_point_right = ticks_right;
    unlock();
}

void update_target_by(float translation_meters, float rotation_radians) {
    float ticks_left, ticks_right;
    std::tie(ticks_left, ticks_right)
        = localization::convert_to_encoder_ticks(translation_meters, rotation_radians);

    lock();
    set_point_left += ticks_left;
    set_point_right += ticks_right;
    unlock();
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
