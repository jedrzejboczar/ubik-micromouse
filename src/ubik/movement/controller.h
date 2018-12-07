#pragma once

#include <algorithm>

#include "FreeRTOS.h"
#include "task.h"

#include "trajectory.h"
#include "moves.h"
#include "regulator.h"
#include "correction.h"

/*
 * Controller for executing moves on the robot.
 * It also updates set-point according to current correction policies.
 *
 * Usage:
 * The controller task has a queue to which we can send Moves (if queue
 * is full, we block for portMAX_DELAY), and the function move() returns
 * the ID assigned to the moved sent.
 * The controller processes moves from the queue sequentially. The caller
 * task can continue execution, and can synchronise with the controller
 * using wait_until_finished() by specifying ID of the move that has been
 * sent before.
 *
 * In the current implementation only one task can use this module at a time.
 * It probably doesn't make sense for more tasks to pass moves anyway as
 * the results would just be a chaos.
 * This policy is enforced by requireing a task to become_owner() (and optionally
 * release() when we change the task that will send moves).
 * For checks, configASSERT is used, but, for simplicity, it is not 100% thread
 * safe, so be carreful (although any problems should be very unlikely).
 *
 *
 * Why separate task?
 *   Because we can provide moves and work on next ones while they are executed.
 * Why queue?
 * Because we can make sure that there are no delays between subsequent moves
 * (just always push one more move to the task, so that it always have a next
 * move to be done). This is important for moves that end with non-zero velocity.
 *
 * WARNINGS:
 *  - be carreful for MOVES_QUEUE_LENGTH, if e.g. it is equal to 2 and we pass
 *    4 moves and wait for the first one, in reality this move has been finished
 *    much earlier (because the task had to block on the queue until it could
 *    send the last move)
 *  - using Line(vel_final != 0) and then Rotate has undefined behaviour
 *    (as we end with a velocity which interpretation changes from linear
 *    to angular in one moment for the next move)
 */


namespace movement::controller {

constexpr float LOOP_FREQUENCY = 100;
static constexpr size_t MOVES_QUEUE_LENGTH = 3;

typedef uint32_t MoveId;

void initialise();

void become_owner();
void release();

MoveId move(AnyMove &&next_move);
void wait_until_finished(MoveId id);

void controller_task(void *);


} // namespace movement::controller
