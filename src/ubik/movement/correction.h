#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "ubik/common/robot_parameters.h"
#include "ubik/common/integrator.h"
#include "ubik/common/distance_sensors.h"
#include "ubik/logging/logging.h"
#include "ubik/movement/regulator.h"


namespace movement::correction {

namespace ds = distance_sensors;

std::pair<float, float> next_position_delta(float dt);


class Corrector {
    bool enabled = false;
    bool calibrated = false;

    friend std::pair<float, float> next_position_delta(float dt);
protected:
    virtual uint8_t sensors_used() = 0;
    virtual bool perform_calibration() = 0;
    virtual void update(const ds::Readings &readings, float &vel_lin, float &vel_ang) = 0;

    // perform `n_loops` measurements after each `dt_ms` miliseconds for `sensors_used()`,
    // save average results in `results`, returns number of loops with ok readings
    // (other loops are not taken into account when calculating average)
    int measure_average(ds::Readings &results, const int n_loops, const int dt_ms);

public:
    bool calibrate();
    void set_enabled(bool enabled);

    bool is_calibrated() { return calibrated; }
    bool is_enabled() { return enabled; }
};

extern Corrector &front_walls;
extern Corrector &side_walls;

} // namespace movement::correction
