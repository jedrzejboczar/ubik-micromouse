#include "correction.h"

namespace movement::correction {

namespace ds = distance_sensors;


// correction values for to-side-walls-correction
namespace side_walls {

constexpr int LEFT = 5;
constexpr int RIGHT = 0;
constexpr auto SENSORS = spi::gpio::DISTANCE_SENSORS[LEFT] | spi::gpio::DISTANCE_SENSORS[RIGHT];

// minimum values for the readings to be considered valid
constexpr int16_t LEFT_MIN = 1500;
constexpr int16_t RIGHT_MIN = 1500;

constexpr float GAIN = 0.5;
static const float max_vel_ang = constants::deg2rad(90);

static bool enabled = false;
static int16_t calibration_left = -1;
static int16_t calibration_right = -1;

static bool readings_valid(const ds::Readings readings) {
    return readings.ok(SENSORS) && // <- not really needed
        (readings.sensor[LEFT] > LEFT_MIN && readings.sensor[RIGHT] > RIGHT_MIN);
}

bool calibrate() {
    const int n_loops = 10;
    const int dt_ms = 10;
    int counter = 0;
    int32_t sum_left = 0, sum_right = 0;
    for (int i = 0; i < n_loops; i++) {
        // check `i` to avoid unnecessary waiting
        if (i != 0) vTaskDelay(pdMS_TO_TICKS(dt_ms));
        // measure
        auto readings = ds::read(SENSORS);
        if (readings_valid(readings))
        {
            sum_left += readings.sensor[LEFT];
            sum_right += readings.sensor[RIGHT];
            counter++;
        }
    }
    // require at least half loops to succeed
    if (counter < n_loops / 2)
        return false;
    // caluclate the mean values and set them
    calibration_left = sum_left /  counter;
    calibration_right = sum_right / counter;

    logging::printf(100, "calib_l = %d, calib_r = %d\n",
            calibration_left, calibration_right);

    return true;
}

void set_enabled(bool e) {
    if (calibration_left < 0 || calibration_right < 0) {
        logging::printf(80, "[# ERROR #] [corr] Cannot enable side-walls correction - not calibrated!\n");
        return;
    }
    enabled = e;
}

static void update(const ds::Readings &readings, float &, float &vel_ang) {
    if (! readings_valid(readings))
        return;

    // apply calibration scaling
    configASSERT(calibration_left > 0 && calibration_right > 0);
    float left = static_cast<float>(readings.sensor[LEFT]) / calibration_left;
    float right = static_cast<float>(readings.sensor[RIGHT]) / calibration_right;

    // now left, right should be (1.0, 1.0) at the calbiration point
    // and being closer to the right wall increases `right`
    // calculte the difference such that it is possitive if we should turn left
    float diff = right - left;
    // calculate the velocity and saturate it and update
    float v_ang = GAIN * diff * max_vel_ang;
    v_ang = std::min(v_ang, max_vel_ang);
    v_ang = std::max(v_ang, -max_vel_ang);
    vel_ang += v_ang;

    logging::printf(200, "left = %12f, right = %12f, diff = %12f, v_ang = %12f\n",
            left, right, diff, v_ang);
}

} // namespace side_walls

std::pair<float, float> calculate_correction_velocities() {
    // check which sensors we have to read
    uint8_t sensors = 0;
    if (side_walls::enabled) {
        sensors |= side_walls::SENSORS;
    }

    // read the required sensors once, this saves some time
    auto readings = ds::read(sensors);

    // update the correction velocities from all sources
    float vel_lin = 0, vel_ang = 0;
    if (side_walls::enabled)
        side_walls::update(readings, vel_lin, vel_ang);

    return std::make_pair(vel_lin, vel_ang);
}




} // namespace movement::correction
