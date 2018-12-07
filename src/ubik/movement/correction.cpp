#include "correction.h"

namespace movement::correction {

namespace ds = distance_sensors;

// perform `n_loops` measurements each `dt_ms` miliseconds for `sensors`,
// save average results in `results`, returns number of loops with ok readings
// (other loops are not taken into account when calculating average)
static int measure_average(ds::Readings &results, uint8_t sensors, const int n_loops, const int dt_ms) {
    int counter = 0;
    std::array<int32_t, spi::gpio::N_DISTANCE_SENSORS> sums = {0};
    results = {-1, -1, -1, -1, -1, -1};

    for (int i = 0; i < n_loops; i++) {
        // check `i` to avoid unnecessary waiting
        if (i != 0) vTaskDelay(pdMS_TO_TICKS(dt_ms));
        // measure
        auto readings = ds::read(sensors);
        if (readings.ok(sensors))
        {
            counter++;
            for (size_t j = 0; j < spi::gpio::N_DISTANCE_SENSORS; j++)
                if ((sensors & spi::gpio::DISTANCE_SENSORS[j]) != 0)
                    sums[j] += readings.sensor[j];
        }
    }

    // no calculations if no loops were ok
    if (counter == 0)
        return counter;

    // calculate the mean values
    for (size_t i = 0; i < spi::gpio::N_DISTANCE_SENSORS; i++)
        if ((sensors & spi::gpio::DISTANCE_SENSORS[i]) != 0)
            results.sensor[i] = sums[i] / counter;

    logging::printf(100, "[corr] Calibration: %5d %5d %5d %5d %5d %5d\n",
            results.sensor[0], results.sensor[1], results.sensor[2],
            results.sensor[3], results.sensor[4], results.sensor[5]);

    return counter;

}


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
    ds::Readings calib;
    int n_valid = measure_average(calib, SENSORS, 10, 10);
    if (n_valid <= 5)
        return false;

    // check if the readings are in good range for walls
    if (calib.sensor[LEFT] < LEFT_MIN || calib.sensor[RIGHT] < RIGHT_MIN)
        return false;

    calibration_left = calib.sensor[LEFT];
    calibration_right = calib.sensor[RIGHT];

    logging::printf(100, "sides: calib_l = %d, calib_r = %d\n",
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

    // logging::printf(200, "left = %12f, right = %12f, diff = %12f, v_ang = %12f\n",
    //         left, right, diff, v_ang);
}

} // namespace side_walls


namespace front_walls {

constexpr int LEFT = 4;
constexpr int RIGHT = 1;
constexpr auto SENSORS = spi::gpio::DISTANCE_SENSORS[LEFT] | spi::gpio::DISTANCE_SENSORS[RIGHT];

// minimum values for the readings to be considered valid
constexpr int16_t LEFT_MIN = 1000;
constexpr int16_t RIGHT_MIN = 1000;

constexpr float GAIN_LIN = 0.5;
static const float max_vel_lin = 0.4;
constexpr float GAIN_ANG = 0.5;
static const float max_vel_ang = constants::deg2rad(270);

static bool enabled = false;
static int16_t calibration_left = -1;
static int16_t calibration_right = -1;

static bool readings_valid(const ds::Readings readings) {
    return readings.ok(SENSORS) && // <- not really needed
        (readings.sensor[LEFT] > LEFT_MIN && readings.sensor[RIGHT] > RIGHT_MIN);
}

bool calibrate() {
    ds::Readings calib;
    int n_valid = measure_average(calib, SENSORS, 10, 10);
    if (n_valid <= 5)
        return false;

    // check if the readings are in good range for walls
    if (calib.sensor[LEFT] < LEFT_MIN || calib.sensor[RIGHT] < RIGHT_MIN)
        return false;

    calibration_left = calib.sensor[LEFT];
    calibration_right = calib.sensor[RIGHT];

    logging::printf(100, "front: calib_l = %d, calib_r = %d\n",
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

static void update(const ds::Readings &readings, float &vel_lin, float &vel_ang) {
    if (! readings_valid(readings))
        return;

    // apply calibration scaling
    configASSERT(calibration_left > 0 && calibration_right > 0);
    float left = static_cast<float>(readings.sensor[LEFT]) / calibration_left;
    float right = static_cast<float>(readings.sensor[RIGHT]) / calibration_right;

    // now left, right should be (1.0, 1.0) at the calbiration point
    // and being closer to the right wall increases `right`
    // calculte the difference such that it is possitive if we should turn left
    float diff = -(right - left);
    // get mean for forwards/backwards motion
    float mean = (left + right) / 2;
    // calculate the velocity and saturate it and update
    float v_lin = GAIN_LIN * (1.0f - mean) * max_vel_lin;
    float v_ang = GAIN_ANG * diff * max_vel_ang;
    v_lin = std::min(v_lin, max_vel_lin);
    v_lin = std::max(v_lin, -max_vel_lin);
    v_ang = std::min(v_ang, max_vel_ang);
    v_ang = std::max(v_ang, -max_vel_ang);
    vel_lin += v_lin;
    vel_ang += v_ang;

    // logging::printf(200, "left = %12f, right = %12f, v_lin = %12f, v_ang = %12f\n",
    //         left, right, v_lin, v_ang);
}

} // namespace front_walls



std::pair<float, float> calculate_correction_velocities() {
    // check which sensors we have to read
    uint8_t sensors = 0;
    if (side_walls::enabled)
        sensors |= side_walls::SENSORS;
    if (front_walls::enabled)
        sensors |= front_walls::SENSORS;

    // read the required sensors once, this saves some time
    auto readings = ds::read(sensors);

    // update the correction velocities from all sources
    float vel_lin = 0, vel_ang = 0;
    if (side_walls::enabled)
        side_walls::update(readings, vel_lin, vel_ang);
    if (front_walls::enabled)
        front_walls::update(readings, vel_lin, vel_ang);

    // logging::printf(200, "v_lin = %12f, v_ang = %12f\n",
    //         vel_lin, vel_ang);

    return std::make_pair(vel_lin, vel_ang);
}




} // namespace movement::correction
