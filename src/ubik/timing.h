#pragma once

// #define CYCLE_COUNTER_ENABLE_FLOATING_POINT

/*
 * Utilities for measuring processor clock cycles using
 * DWT registers. To get the actual time, divide cycles
 * by system core's clock frequency.
 *
 * Measurement process:
 *    cycles_counter::reset();
 *    cycles_counter::start();
 *    ___MEASURED_CODE___
 *    cycles_counter::stop();
 *    uint32_t cycles = cycles_counter::get();
 *    uint32_t micros = cycles_counter::get_us();
 */

/*
 * Used to obtain Cortex-M4 processor and core peripherals
 * from (already configured) file: #include "core_cmX.h"
 * */
#include "stm32f1xx_hal.h"

namespace cycles_counter {

static inline void reset() {
    // enable trace functionalities (DWT, ITM, ETM, TPIU)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // disable the cycles counter
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    // reset counter value
    DWT->CYCCNT = 0;
}

static inline void start() {
    // enable counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void stop() {
    // only disable the counter
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t get() {
    return DWT->CYCCNT;
}

/*
 * About time range:
 * As the cycle counter is 32-bit it will overflow quite fast,
 * assuming system clock of 72Mhz it overflows after 59.652 seconds.
 *
 * Also we must be careful here when performing calculations on large numbers
 * e.g. maximum value for get_time_ns() at SystemCoreClock=72MHz is 309237645:
 */
// static_assert(309237645u * 1'000'000'000u / 72'000'000u == 4294967291,
//         "This will fail, because multiplication overflows");
// static_assert(309237645ul * 1'000'000'000u / 72'000'000u == 4294967291,
//         "Succeeds, on 64-bit archtectures (my PC)");
static_assert(309237645ull * 1'000'000'000u / 72'000'000u == 4294967291,
        "Succeeds, because we force unsinged long int with 'ul' suffix");
static_assert(static_cast<uint64_t>(309237645u) * 1'000'000'000u / 72'000'000u == 4294967291,
        "Succeeds because we convert to uint64 ()");

// For C++ version < 14 syntax 1'000'000 is illegal, so change it if you need
// nanoseconds needs larger output variable (uint64)
static inline uint64_t get_ns() {
    return static_cast<uint64_t>(get()) * 1'000'000'000u / SystemCoreClock;
}

static inline uint32_t get_us() {
    return static_cast<uint64_t>(get()) * 1'000'000u / SystemCoreClock;
}

static inline uint32_t get_ms() {
    return static_cast<uint64_t>(get()) * 1'000u / SystemCoreClock;
}

#ifdef CYCLE_COUNTER_ENABLE_FLOATING_POINT
static inline float get_sec() {
    return static_cast<float>(get()) / SystemCoreClock;
}
#endif

} // namespace cycles_counter
