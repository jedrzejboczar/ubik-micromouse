#pragma once

#include <algorithm>

#include "FreeRTOS.h"
#include "task.h"

#include "trajectory.h"
#include "moves.h"
#include "regulator.h"
#include "correction.h"

namespace movement::controller {

constexpr float LOOP_FREQUENCY = 100;

/*
 * WARNING:
 * using Line(vel_final != 0) and then Rotate has undefined behavoiur
 * (as we end with a velocity which interpretations changes from linear to
 * angular in one moment)
 */


// TODO: avoid dynamic allocation
void move(Move *m);
void controller_task(void *);


} // namespace movement::controller
