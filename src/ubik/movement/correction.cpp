#include "correction.h"

namespace movement::correction {


void Corrector::set_enabled(bool e) {
    if (e && !calibrated) {
        logging::printf(60, "[# ERROR #] [corr] Cannot enable - not calibrated!\n");
        return;
    }
    enabled = e;
}

bool Corrector::calibrate() {
    calibrated = perform_calibration();
    return calibrated;
}

int Corrector::measure_average(ds::Readings &results, const int n_loops, const int dt_ms) {
    int counter = 0;
    std::array<int32_t, spi::gpio::N_DISTANCE_SENSORS> sums = {0};
    results = {-1, -1, -1, -1, -1, -1};

    for (int i = 0; i < n_loops; i++) {
        // check `i` to avoid unnecessary waiting
        if (i != 0) vTaskDelay(pdMS_TO_TICKS(dt_ms));
        // measure
        auto readings = ds::read(sensors_used());
        if (readings.ok(sensors_used()))
        {
            counter++;
            for (size_t j = 0; j < spi::gpio::N_DISTANCE_SENSORS; j++)
                if ((sensors_used() & spi::gpio::DISTANCE_SENSORS[j]) != 0)
                    sums[j] += readings.sensor[j];
        }
    }

    // no calculations if no loops were ok
    if (counter == 0)
        return counter;

    // calculate the mean values
    for (size_t i = 0; i < spi::gpio::N_DISTANCE_SENSORS; i++)
        if ((sensors_used() & spi::gpio::DISTANCE_SENSORS[i]) != 0)
            results.sensor[i] = sums[i] / counter;

    // logging::printf(100, "[corr] Calibration: %5d %5d %5d %5d %5d %5d\n",
    //         results.sensor[0], results.sensor[1], results.sensor[2],
    //         results.sensor[3], results.sensor[4], results.sensor[5]);

    return counter;
}

/******************************************************************************/

class SideWallsCorrector: public Corrector {
    static constexpr int LEFT = 5;
    static constexpr int RIGHT = 0;

    // minimum values for the readings to be considered valid
    static constexpr int16_t LEFT_MIN = 1500;
    static constexpr int16_t RIGHT_MIN = 1500;

    static constexpr float GAIN_ANG = 0.5;
    const float max_vel_ang = constants::deg2rad(90);

    int16_t calib_left = -1;
    int16_t calib_right = -1;

    virtual uint8_t sensors_used() override {
        return spi::gpio::DISTANCE_SENSORS[LEFT] | spi::gpio::DISTANCE_SENSORS[RIGHT];
    }

    virtual bool perform_calibration() override;
    virtual void update(const distance_sensors::Readings &readings, float &vel_lin, float &vel_ang) override;
};

bool SideWallsCorrector::perform_calibration() {
    ds::Readings calib;
    int n_valid = measure_average(calib, 10, 10);
    if (n_valid <= 5)
        return false;

    // check if the readings are in good range for walls
    if (calib.sensor[LEFT] < LEFT_MIN || calib.sensor[RIGHT] < RIGHT_MIN)
        return false;

    calib_left = calib.sensor[LEFT];
    calib_right = calib.sensor[RIGHT];
    return true;
}

void SideWallsCorrector::update(const distance_sensors::Readings &readings, float &vel_lin, float &vel_ang) {
    if (readings.sensor[LEFT] < LEFT_MIN || readings.sensor[RIGHT] < RIGHT_MIN)
        return;

    // apply calibration scaling
    float left = static_cast<float>(readings.sensor[LEFT]) / calib_left;
    float right = static_cast<float>(readings.sensor[RIGHT]) / calib_right;

    // now left, right should be (1.0, 1.0) at the calbiration point
    // and being closer to the right wall increases `right`
    // calculte the difference such that it is possitive if we should turn left
    float diff = right - left;
    // calculate the velocity and saturate it and update
    float v_ang = GAIN_ANG * diff * max_vel_ang;
    v_ang = std::min(v_ang, max_vel_ang);
    v_ang = std::max(v_ang, -max_vel_ang);
    vel_ang += v_ang;
}

/******************************************************************************/

class FrontWallsCorrector: public Corrector {
    static constexpr int LEFT = 4;
    static constexpr int RIGHT = 1;

    // minimum values for the readings to be considered valid
    static constexpr int16_t LEFT_MIN = 800;
    static constexpr int16_t RIGHT_MIN = 800;

    static constexpr float GAIN_LIN = 0.7;
    static constexpr float GAIN_ANG = 0.7;
    const float max_vel_lin = 0.3;
    const float max_vel_ang = constants::deg2rad(270);

    int16_t calib_left = -1;
    int16_t calib_right = -1;

    virtual uint8_t sensors_used() override {
        return spi::gpio::DISTANCE_SENSORS[LEFT] | spi::gpio::DISTANCE_SENSORS[RIGHT];
    }

    virtual bool perform_calibration() override;
    virtual void update(const distance_sensors::Readings &readings, float &vel_lin, float &vel_ang) override;
};

bool FrontWallsCorrector::perform_calibration() {
    ds::Readings calib;
    int n_valid = measure_average(calib, 10, 10);
    if (n_valid <= 5)
        return false;

    // check if the readings are in good range for walls
    if (calib.sensor[LEFT] < LEFT_MIN || calib.sensor[RIGHT] < RIGHT_MIN)
        return false;

    calib_left = calib.sensor[LEFT];
    calib_right = calib.sensor[RIGHT];
    return true;
}

void FrontWallsCorrector::update(const distance_sensors::Readings &readings, float &vel_lin, float &vel_ang) {
    if (readings.sensor[LEFT] < LEFT_MIN || readings.sensor[RIGHT] < RIGHT_MIN)
        return;

    // apply calibration scaling
    float left = static_cast<float>(readings.sensor[LEFT]) / calib_left;
    float right = static_cast<float>(readings.sensor[RIGHT]) / calib_right;

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
}

/******************************************************************************/

static FrontWallsCorrector front_corr;
static SideWallsCorrector side_corr;

Corrector &front_walls = front_corr;
Corrector &side_walls = side_corr;

Corrector *correctors[] = {&front_walls, &side_walls};

std::pair<float, float> next_position_delta(float dt) {
    static Integrator i_lin, i_ang;

    // check which sensors we have to read
    uint8_t sensors = 0;
    for (auto corr: correctors)
        if (corr->is_enabled())
            sensors |= corr->sensors_used();

    // read the required sensors once, this saves some time
    auto readings = ds::read(sensors);

    // update the correction velocities from all sources
    float vel_lin = 0, vel_ang = 0;
    for (auto corr: correctors)
        if (corr->is_enabled())
            corr->update(readings, vel_lin, vel_ang);

    return std::make_pair(i_lin.next_as_delta(vel_lin, dt), i_ang.next_as_delta(vel_ang, dt));
}


} // namespace movement::correction
