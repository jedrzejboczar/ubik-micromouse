#include "controller.h"

namespace movement::controller {

/*
 * We pass a move with its ID to the queue and return this ID from move().
 * Doing so we can then wait for this ID to be finished.
 * Move 0 is used for the moment when no move has been sent yet.
 * The first actual move ID is 1.
 */
struct MoveWithId {
    MoveId id;
    AnyMove move;
};

constexpr MoveId first_move_id = 1;
static MoveId next_move_id = first_move_id;
static MoveId last_finished_move_id = 0;


static QueueHandle_t moves_queue = nullptr;
static TaskHandle_t caller_task = nullptr;

void initialise() {
    moves_queue = xQueueCreate(MOVES_QUEUE_LENGTH, sizeof(MoveWithId));
    configASSERT(moves_queue != nullptr);
}

// TODO: are these two thread safe? like 100%? probably not...
// succeeds only if there is no owner
void become_owner() {
    configASSERT(caller_task == nullptr);
    caller_task = xTaskGetCurrentTaskHandle();
}
void release() {
    configASSERT(caller_task != nullptr);
    caller_task = nullptr;
}


MoveId move(AnyMove &&next_move) {
    // only one task can use controller API
    auto current_task = xTaskGetCurrentTaskHandle();
    configASSERT(current_task == caller_task);

    // create the data to be sent
    MoveId id = next_move_id++;
    // MoveWithId data = {next_move_id++, std::forward<AnyMove>(next_move)};
    MoveWithId data = {id, next_move};

    // should always succeed
    bool success = xQueueSend(moves_queue, &data, portMAX_DELAY) == pdPASS;
    configASSERT(success);

    return id;
}

void wait_until_finished(MoveId id) {
    // assert that any move has been already added
    configASSERT(next_move_id > first_move_id);
    // will exit instantly if the id is lower
    while (id > last_finished_move_id)
        vTaskDelay(5);
}

void controller_task(void *) {
    constexpr float dt = 1.0f / LOOP_FREQUENCY;

    // a base for generating trajectory
    TrajectoryGenerator trajectory;

    // For now I don't have a better idea, this just works (because
    // TrajectoryGenerator will ignore moves with 0 distance).
    // We could redefine AnyMove to std::variant<std::monostate, Line, ...>,
    // but that kinda sucks, as user can then pass empty variants.
    MoveId current_move_id = 0;
    MoveWithId move_requested = {current_move_id, Line(0, 0, 0, 0)};

    auto last_start = xTaskGetTickCount();
    while (1) {
        // used to apply both updates in one call to update_target_by()
        Pair final_position_update = {0, 0};

        // if we finished update the "global" variable
        if (trajectory.has_finished() && last_finished_move_id != current_move_id) {
            last_finished_move_id = current_move_id;
        }

        // if the current move is finished, then try to get the next one
        if (trajectory.has_finished() && xQueueReceive(moves_queue, &move_requested, 0) == pdPASS) {
            current_move_id = move_requested.id;
            // initialise the received move
            std::visit([&trajectory](auto &move) { move.initialise_generator(trajectory); },
                    move_requested.move);
            // logging::printf(60, "[ctrl] Started move: %d\n", current_move_id);
        }

        // if the move is ongoing, process next step (also test to ignore the first "artificial" move)
        if (!trajectory.has_finished() && current_move_id > 0) {
            // calculate next generator output
            float delta_pos = trajectory.next_position_delta(dt);

            // get the position updated according to the logic of this move
            Pair position_update = std::visit(
                    [&delta_pos](auto &move) { return move.convert_position(delta_pos); },
                    move_requested.move);

            // store the update
            final_position_update.first += position_update.first;
            final_position_update.second += position_update.second;
        }

        // apply correction independently of the main moves controller
        Pair correction_update = correction::next_position_delta(dt);
        final_position_update.first += correction_update.first;
        final_position_update.second += correction_update.second;

        // update regulator set-point
        std::apply(regulator::update_target_by, final_position_update);

        // keep oncstant frequency
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(1000 / LOOP_FREQUENCY));
    }
}






} // namespace movement::controller
