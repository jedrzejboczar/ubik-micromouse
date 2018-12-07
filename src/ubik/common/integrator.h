#pragma once

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

    // this is useful e.g. when we integrate velocity, but want to get position updates as delta
    float next_as_delta(float current_value, float dt) {
        float last_output = output;
        return next(current_value, dt) - last_output;
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

