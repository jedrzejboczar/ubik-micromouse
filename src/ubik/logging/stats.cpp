#include "stats.h"

#include <cstring>
#include <algorithm>

#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"


/*** Timer for FreeRTOS runtime stats *****************************************/

// we use the HAL "system timer"
// HAL configures the timer's prescaler for 1MHz clock
// and tick for 1kHz.
// FreeRTOS holds it's time in uint32 so it will work for:
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

/*** External variables from different modules ********************************/

namespace logging {
extern uint32_t logs_lost;
extern uint32_t logs_lost_from_uart_errors;
extern uint32_t logs_lost_from_notification_timeouts;
extern uint32_t logs_lost_from_isr;
}

namespace movement::regulator {
extern float mean_regulation_time_us;
}
using movement::regulator::mean_regulation_time_us;

namespace spi {
extern float mean_read_encoders_time_us;
extern float mean_update_pins_time_us;
}
using spi::mean_read_encoders_time_us;
using spi::mean_update_pins_time_us;


/*** Formats for printing final stats *****************************************/

namespace logging {

static const char fmt_header[] = "\
=== STATS =================================\n";

static const char fmt_memory[] = "\
------- MEMORY ---------------------------\n\
free heap: %d\n\
minimum free heap: %d\n\
free stack from here: %lu\n";

static const char fmt_state[] = "\
------- STATE ----------------------------\n\
Name           \tState\tPrio\tStack\tNr\n";

static const char fmt_runtime[] = "\
------- RUNTIME --------------------------\n\
Name           \tTime\t\tPercent\n";

static const char fmt_counts[] = "\
------- COUNTS ---------------------------\n\
logs_lost = %lu\n\
logs_lost_from_uart_errors = %lu\n\
logs_lost_from_notification_timeouts = %lu\n\
logs_lost_from_isr = %lu\n\
mean_regulation_time_us = %ld\n\
mean_read_encoders_time_us = %ld\n\
mean_update_pins_time_us = %ld\n\
";

static const char fmt_footer[] =
"===========================================\n";

/*** Implementation of stats printing *****************************************/

void stats_monitor_task(void *ticks_to_wait_as_ptr) {
    if (ticks_to_wait_as_ptr == nullptr) {
        const char msg[] = "[# ERROR #] system_monitor: ticks_to_wait_as_ptr=nullptr! suspending...\n";
        logging::log(logging::Msg::from_static(msg), true);
        vTaskSuspend(nullptr);
    }
    BaseType_t ticks_to_wait = reinterpret_cast<BaseType_t>(ticks_to_wait_as_ptr);

    while (1) {
        vTaskDelay(ticks_to_wait);
        print_stats();
    }
}

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
        // start stats
        logging::log_blocking(logging::Msg::from_static(fmt_header));

        // memory information
        auto msg = logging::Msg::dynamic(strlen(fmt_memory) + 3*10);
        snprintf(msg.as_chars(), msg.size, fmt_memory,
                xPortGetFreeHeapSize(),
                xPortGetMinimumEverFreeHeapSize(),
                free_stack);
        logging::log_blocking(msg);

        // state (replace carriage return)
        // header
        logging::log_blocking(logging::Msg::from_static(fmt_state));
        // contents
        msg = logging::Msg::dynamic(4 * 60);
        vTaskList(msg.as_chars());
        std::replace(msg.data, msg.data + strlen(msg.as_chars()), '\r', ' ');
        logging::log_blocking(msg);

        // runtime (replace carriage return)
        // header
        logging::log_blocking(logging::Msg::from_static(fmt_runtime));
        // contents
        msg = logging::Msg::dynamic(4 * 60);
        vTaskGetRunTimeStats(msg.as_chars());
        std::replace(msg.data, msg.data + strlen(msg.as_chars()),
                '\r', ' ');
        logging::log_blocking(msg);

        // error counts
        msg = logging::Msg::dynamic(strlen(fmt_counts) + 7*10);
        snprintf(msg.as_chars(), msg.size, fmt_counts,
                logs_lost,
                logs_lost_from_uart_errors,
                logs_lost_from_notification_timeouts,
                logs_lost_from_isr,
                static_cast<int32_t>(mean_regulation_time_us),
                static_cast<int32_t>(mean_read_encoders_time_us),
                static_cast<int32_t>(mean_update_pins_time_us)
                );
        logging::log_blocking(msg);

        // end stats
        logging::log_blocking(logging::Msg::from_static(fmt_footer));
    }
    logging::unlock();
}

} // namespace logging
