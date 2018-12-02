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



// temporary, this should be in controller_task
static Move *current_move = nullptr;

// TODO: it's a temporary implementation
void move(Move *m) {
    configASSERT(m != nullptr);
    configASSERT(current_move == nullptr);

    current_move = m;
    while (current_move != nullptr)
        vTaskDelay(10);
}

void controller_task(void *) {
    constexpr float dt = 1.0f / LOOP_FREQUENCY;

    TrajectoryGenerator trajectory;
    bool initialised = false;

    // velocity -> position
    Integrator corr_integrator_lin, corr_integrator_ang;
    // position -> delta_position
    Pair corr_last_position = {0, 0};

    auto last_start = xTaskGetTickCount();
    while (1) {

        // update position from current move it it exists
        if (current_move != nullptr) {
            // initialise next move  if needed
            if (!initialised) {
                logging::printf(50, "Initialising move\n");
                initialised = true;
                current_move->initialise_generator(trajectory);
            }

            // calculate next generator output
            float delta_pos = trajectory.next_position_delta(dt);

            // get the position updated according to the logic of this move
            Pair position_update = current_move->convert_position(delta_pos);
            // update regulator set-point
            std::apply(regulator::update_target_by, position_update);
        }

        // check if we should prepare for next move
        if (current_move != nullptr && trajectory.has_finished()) {
            logging::printf(50, "Move finished\n");
            delete current_move;
            current_move = nullptr;
            initialised = false;
        }

#if 0
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
        std::apply(regulator::update_target_by, position_delta);
#endif

        // keep oncstant frequency
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(1000 / LOOP_FREQUENCY));
    }
}






} // namespace movement::controller
