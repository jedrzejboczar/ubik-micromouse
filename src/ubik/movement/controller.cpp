#include "controller.h"

/*
 * Class for numerical integration of a time signal.
 * Uses the trapeziodal rule to increase the accurency.
 */
class Integrator {
    float output;
    float last_value;
    bool should_reset_state = false;

public:
    Integrator(float initial_state=0, float last_value=0) {
        reset(initial_state, last_value);
    }

    void reset(float initial_state=0, float last_value=0) {
        this->output = initial_state;
        this->last_value = last_value;
    }

    // calculate the next value of output
    float next(float current_value, float dt) {

        // trapezoidal rule: (a+b) / 2 * h
        output += (current_value + last_value) / 2 * dt;
        last_value = current_value;

        return output;
    }
};

// // NOT MY CPP-FU LEVEL
// // this wont' work because we cannot convert pair<float, float> to array<float, 2>
// Integrators<2> corr_vel_integrator;
// position_correction = corr_vel_integrator.next(correction_vel, dt);
// template<size_t N>
// class Integrators {
//     typedef std::array<float, N> float_array_t;
//     std::array<Integrator, N> ints;
// public:
//     Integrators(float_array_t initial_state={0}, float_array_t last_value={0}) {
//         reset(initial_state, last_value);
//     }
//
//     void reset(float_array_t initial_state={0}, float_array_t last_value={0}) {
//         for (int i = 0; i < N; i++)
//             ints[i].reset(initial_state[i], last_value[i]);
//     }
//
//     // calculate the next value of output
//     float_array_t next(float_array_t current_value, float dt) {
//         float_array_t output;
//         for (int i = 0; i < N; i++)
//             output[i] = ints[i].next(current_value[i], dt);
//         return output;
//     }
// };



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

    TrajectoryGenerator trajectory;

    // velocity -> position
    Integrator corr_integrator_lin, corr_integrator_ang;
    // position -> delta_position
    Pair corr_last_position = {0, 0};

    // For now I don't have a better idea, this just works (because
    // TrajectoryGenerator will ignore moves with 0 distance).
    // We could redefine AnyMove to std::variant<std::monostate, Line, ...>,
    // but that kinda sucks, as user can then pass empty variants.
    MoveId current_move_id = 0;
    MoveWithId move_requested = {current_move_id, Line(0, 0, 0, 0)};

    auto last_start = xTaskGetTickCount();
    while (1) {

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
        }

        // if the move is ongoing, process next step (also test to ignore the first "artificial" move)
        if (!trajectory.has_finished() && current_move_id > 0) {
            // calculate next generator output
            float delta_pos = trajectory.next_position_delta(dt);

            // get the position updated according to the logic of this move
            Pair position_update = std::visit(
                    [&delta_pos](auto &move) { return move.convert_position(delta_pos); },
                    move_requested.move);

            // update regulator set-point
            std::apply(regulator::update_target_by, position_update);
        }

        // apply correction independently
        Pair correction_vel = correction::calculate_correction_velocities();
        Pair position = {
            corr_integrator_lin.next(correction_vel.first, dt),
            corr_integrator_ang.next(correction_vel.second, dt)};
        Pair position_delta = {
            position.first - corr_last_position.first,
            position.first - corr_last_position.first};
        corr_last_position = position;
        // update regulator set-point TODO: do both in one call
        // std::apply(regulator::update_target_by, position_delta);
        // if (position_delta.first != 0 || position_delta.second != 0)
            // logging::printf(60, "corr = %12.6f %12.6f\n", position_delta.first, position_delta.second);

        // keep oncstant frequency
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(1000 / LOOP_FREQUENCY));
    }
}






} // namespace movement::controller
