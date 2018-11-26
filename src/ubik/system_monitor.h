#pragma once

#include <tuple>

#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

namespace system_monitor {

// system monitor task loop frequency
static constexpr float LOOP_FREQ_HZ = 10;
static constexpr float VOLTAGE_WARINGS_FREQ_HZ = 1;

// system monitor button regulation toggling
static constexpr bool REGULATION_START_ON = true; // state in which to start robot


// button debouncing
static constexpr int N_BUTTON_MEASUREMENTS = 10; // how many samples to take
static constexpr int MIN_BUTTON_ON_COUNT = 7; // how many samples are needed to register button press
static constexpr int TIME_BETWEEN_MEASUREMENTS_MS = 6; // time between subsequent samples
static constexpr int MIN_BUTTON_DELAY_MS = 500; // minimum delay between subsequent button presses


// voltage & current measurements
constexpr bool IS_BATTERY_SUPPLY = true;
constexpr int PRINT_BATTERY_MEASUREMENTS_EVERY = 100 /* 30 */; // every that many loops
constexpr float MAX_ADC_READING = ((1 << 12) - 1);
constexpr float VCC = 3.3;
constexpr float VOLTAGE_RESISTOR_DIVIDER = 10.0 / 32.0;
constexpr float CURRENT_READING_GAIN = 11;
constexpr float CURRENT_MEASUREMENT_RESISTOR = 0.1;

// li-pol battery constants
constexpr int N_LIPOL_CELLS = 2;
constexpr float CRITICAL_VOLTAGE = 3.1 * N_LIPOL_CELLS;
constexpr float WARING_VOLTAGE = 3.4 * N_LIPOL_CELLS;
static_assert(WARING_VOLTAGE > CRITICAL_VOLTAGE);

// IIR filter coefficients
// in fact this variation of: y = a*x + (1-a)*y,
// but rather: y[n] = b/2 * x[n] + b/2 * x[n-1] + a*y[n-1]
constexpr float VOLTAGE_IIR_B[] = {0.08636403, 0.08636403};
constexpr float VOLTAGE_IIR_A[] = {-0.82727195};


void initialise();

bool get_regulation_state();

/*
 * The system monitor by default uses button for toggling motors regulation.
 * To use the button for other tasks ALWAYS lock it first, this will disable
 * regulation toggling. Remember to unlock it later!
 * No public function in this module will lock the button by itself!
 */
void lock_button();
void unlock_button();
bool is_button_locked();

/*
 * Wait for a specified time until user pressed the button,
 * return false if not pressed.
 * Uses button debouncing, so minimal wait time =
 *     N_BUTTON_MEASUREMENTS * TIME_BETWEEN_MEASUREMENTS_MS
 */
bool wait_for_button_press(uint32_t max_wait_time_ms);

/*
 * Select a value from continuous range from `min_value` to `max_value`, where
 * full wheel turn (360 deg.) for each wheel corresponds to a change of
 * `value_per_left_wheel_turn` or 'value_per_right_wheel_turn'.
 * Forward motion of a wheel increases the value.
 * To confirm the chosen value, press the button.
 * Starts from `initial_value`.
 * If `print_prompt` == nullptr, printing is disabled.
 */
static constexpr float SELECTION_RIGHT_WHEEL_RESOLUTION_RATIO = 10;  // right wheel has N times finer resolution
static constexpr float MIN_PRINTED_DIFFERENCE_RATIO = 0.0005;  // ratio of full range
float select_with_wheels(float initial_value, std::pair<float, float> values_range,
        float change_per_left_wheel_turn, const char *print_prompt="Selecting...");
// right wheel increments by 1, always
int select_with_wheels_int(int initial_value, std::pair<int, int> values_range,
        int increments_per_left_wheel_turn, const char *print_prompt="Selecting...");


void system_monitor_task(void *);

} // namespace system_monitor
