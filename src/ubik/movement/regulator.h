#pragma once

namespace movement {
namespace regulator {


void regulation_task(void *);

// set the absolute value of current regulation target
void set_regulation_target(float translation_meters, float rotation_radians);
// modify the value of current regulation target by given amouts (more practical)
void update_regulation_target(float translation_meters, float rotation_radians);


} // namespace regulator
} // namespace movement
