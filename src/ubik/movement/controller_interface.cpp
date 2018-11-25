#include "controller.h"
#include "regulator.h"

// Interface implementation for the controller.
// This just forwards the call to the regulator.
void movement::controller::update_target_by(float distance_linear, float distance_angular) {
    regulator::update_target_by(distance_linear, distance_angular);
}

void movement::controller::delay(float dt) {
    vTaskDelay(pdMS_TO_TICKS(1e3f * dt));
}
