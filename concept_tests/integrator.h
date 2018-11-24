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
};
