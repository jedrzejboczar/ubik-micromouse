#include "controller.h"
#include "regulator.h"
#include "correction.h"

// Interface implementation for the controller.
namespace movement::controller {

// This just forwards the call to the regulator.
void update_target_by(float distance_linear, float distance_angular) {
    regulator::update_target_by(distance_linear, distance_angular);
}

void delay(float dt) {
    vTaskDelay(pdMS_TO_TICKS(1e3f * dt));
}


// in this approach the controller doesn't know that we change set point
void apply_correction(float dt) {
    float corr_vel_lin, corr_vel_ang;
    std::tie(corr_vel_lin, corr_vel_ang) = correction::calculate_correction_velocities();
    update_target_by(
            corr_vel_lin * dt,
            corr_vel_ang * dt);
}

} // namespace movement::controller
