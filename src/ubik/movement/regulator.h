#pragma once

namespace movement {
namespace regulator {


void regulation_task(void *);

void initialise();
void set_regulation_target(float translation, float rotation);


} // namespace regulator
} // namespace movement
