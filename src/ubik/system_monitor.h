#pragma once

#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

// system monitor task loop frequency
static constexpr float LOOP_FREQ_HZ = 10;
static constexpr float VOLTAGE_WARINGS_FREQ_HZ = 1;

// button debouncing
static constexpr int N_BUTTON_MEASUREMENTS = 10; // how many samples to take
static constexpr int MIN_BUTTON_ON_COUNT = 6; // how many samples are needed to register button press
static constexpr int TIME_BETWEEN_MEASUREMENTS_MS = 8; // time between subsequent samples
static constexpr int MIN_BUTTON_DELAY_MS = 500; // minimum delay between subsequent button presses

// voltage measurements
constexpr bool IS_BATTERY_SUPPLY = false;
constexpr float MAX_ADC_READING = ((1 << 12) - 1);
constexpr float VOLTAGE_RESISTOR_DIVIDER = 10.0 / 32.0;
constexpr float VCC = 3.3;
// li-pol battery constants
constexpr int N_LIPOL_CELLS = 2;
constexpr float CRITICAL_VOLTAGE = 3.1 * N_LIPOL_CELLS;
constexpr float WARING_VOLTAGE = 3.4 * N_LIPOL_CELLS;
// IIR filter coefficients
// in fact this variation of: y = a*x + (1-a)*y,
// but rather: y[n] = b/2 * x[n] + b/2 * x[n-1] + a*y[n-1]
constexpr float VOLTAGE_IIR_B[] = {0.08636403, 0.08636403};
constexpr float VOLTAGE_IIR_A[] = {-0.82727195};

void system_monitor_task(void *);
