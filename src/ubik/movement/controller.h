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
static constexpr size_t MOVES_QUEUE_LENGTH = 5;

/*
 * WARNING:
 * using Line(vel_final != 0) and then Rotate has undefined behavoiur
 * (as we end with a velocity which interpretations changes from linear to
 * angular in one moment)
 */

typedef uint32_t MoveId;

void initialise();

// only one task can use the controller
// first acquire it, if we need to pass ownership, then use release()
// but this is not bullet-proof at this moment
// also, errors hit hard - configASSERT is used
// this is because probably there should be only one task adding moves anyway
// what would happen if many tasks could control movements? probably something bad
void become_owner();
void release();

MoveId move(AnyMove &&next_move);
void wait_until_finished(MoveId id);
void controller_task(void *);


} // namespace movement::controller
