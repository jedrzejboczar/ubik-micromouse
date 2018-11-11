#pragma once

/*
 * PID regulator implemented using the "velocity algorithm",
 * which has less problems with I part (as this algorithm
 * integrates the output, not the error). The I part can
 * cause problems on when we change the I gain while having
 * accumulated error.
 * Uses process variable (PV) instead of SP (set point) for
 * calculating D gain to avoid peaks on rapid SP changes.
 *
 * SP - Set Point - the desired value of the object output
 * PV - Process Variable - the current output of the object
 *
 * TODO: implementation could be optimized if needed
 */
class PID {
    float dt;
    float prev_e = 0;
    float prev_PV = 0;
    float prev2_PV = 0;
    float last_out = 0;
    bool should_reset_state = false;

    void reset(float PV) {
        prev2_PV = prev_PV = PV;
        prev_e = 0;
        should_reset_state = false;
    }
public:
    // gains: proportinal, integral and derivative
    float P = 1, I = 0, D = 0;

    // create PID regulator for the given regulation frequency
    PID(float freq): dt(1.0f / freq) {}

    // request reseting the state on next step
    // this is not done directly to avoid any possible problems
    // with concurency, also needs to know current PV
    void reset_state() {
        should_reset_state = true;
    }

    // calculate the next value of output
    float next(float SP, float PV) {
        if (should_reset_state)
            reset(PV);

        float error = SP - PV;
        float Ti = P/I;
        float Td = D/P;

        // out(t) = out(t-1) + dout/dt
        last_out = last_out + P * ((1 + dt/Ti) * error
                    + (-1) * prev_e
                    - Td/dt * (PV - 2*prev_PV + prev2_PV));

        prev2_PV = prev_PV;
        prev_PV = PV;
        prev_e = error;

        return last_out;
    }
};
