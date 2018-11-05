#include "stats.h"

#include <cstring>
#include <algorithm>

#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

namespace logging {

// we use the HAL "system timer"
// HAL configures the timer's prescaler for 1MHz clock
// and tick for 1kHz.
// FreeRTOS holds its time in uint32 so it will work for:
// Unit  | Max time
// 1ms   | ~7 weeks
// 100us | ~5 days
// 10us  | ~11.2 h
// 1us   | ~1.2 h
extern "C" TIM_HandleTypeDef htim1;
TIM_HandleTypeDef &stats_htim = htim1;

extern "C" void configure_timer_for_runtime_stats(void) { }

extern "C" uint32_t get_runtime_counter_value(void) {
    uint32_t time_us = stats_htim.Instance->CNT;
    uint32_t time_ms = HAL_GetTick();
    uint32_t time_10us = time_ms * 100 + time_us / 10;
    return time_10us;
}

static const char * const stats_fmt1 = "\
=== STATS =================================\n\
------- MEMORY ---------------------------\n\
free heap: %d\n\
minimum free heap: %d\n\
free stack from here: %lu\n\
------- STATE ----------------------------\n\
Name           \tState\tPrio\tStack\tNr\n";

static const char stats_fmt2[] = "\
------- RUNTIME --------------------------\n\
Name           \tTime\t\tPercent\n";

static const char stats_fmt3[] =
"===========================================\n" ;


void print_stats() {

    // This is an estimate of free stack, inaccurate because this task's stack may not be
    // at the end of heap allocated by FreeRTOS. If we could get the end address of freeRTOS
    // heap it would give us more accurate value.
    // Stack grows downwards, so free stack is the difference between SP and RAM base
    // as it's FreeRTOS we have to look at PSP, but overall we should just take the lowset.
    uint32_t stack_pointer = __get_PSP() < __get_MSP() ? __get_PSP() : __get_MSP();
    uint32_t free_stack = stack_pointer - SRAM_BASE;

    // to minimize heap usage, we will lock() for the whole prinitng period
    // and send messages in chunks (as log_blocking() should free the memory
    // before exiting)
    logging::lock();
    {
        // first part
        auto msg = logging::Msg::dynamic(strlen(stats_fmt1) + 3*10);
        snprintf(msg.as_chars(), msg.size, stats_fmt1,
                xPortGetFreeHeapSize(),
                xPortGetMinimumEverFreeHeapSize(),
                free_stack);
        logging::log_blocking(msg);

        // state
        msg = logging::Msg::dynamic(4 * 60);
        vTaskList(msg.as_chars());
        std::replace(msg.data, msg.data + strlen(msg.as_chars()),
                '\r', ' ');
        logging::log_blocking(msg);

        // runtime header
        logging::log_blocking(logging::Msg::from_static(stats_fmt2));

        // runtime stats
        msg = logging::Msg::dynamic(4 * 60);
        vTaskGetRunTimeStats(msg.as_chars());
        std::replace(msg.data, msg.data + strlen(msg.as_chars()),
                '\r', ' ');
        logging::log_blocking(msg);

        // end line
        logging::log_blocking(logging::Msg::from_static(stats_fmt3));
    }
    logging::unlock();
}

} // namespace logging
