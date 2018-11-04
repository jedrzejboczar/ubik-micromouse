#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "logging/logging.h"

void run();
extern "C" void extern_main(void) { run(); }


#include <cstring>
#include <algorithm>

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

const char * const stats_fmt =
"================ STATS ====================\n"
"=== MEMORY:\n"
"free heap: %d\n"
"minimum free heap: %d\n"
// "free stack from here: %lu\n"
"=== STATE:\n"
" Name * State * Priority * Stack * Num\n"
"%s"
"=== RUNTIME:\n"
"%s"
"===========================================\n"
;

void print_system_statistics() {

    // This is an estimate of free stack, inaccurate because this task's stack may not be
    // at the end of heap allocated by FreeRTOS. If we could get the end address of freeRTOS
    // heap it would give us more accurate value.
    // Stack grows downwards, so free stack is the difference between SP and RAM base
    // as it's FreeRTOS we have to look at PSP, but overall we should just take the lowset.
    uint32_t stack_pointer = __get_PSP() < __get_MSP() ? __get_PSP() : __get_MSP();
    uint32_t free_stack = stack_pointer - /*HMCRAMC0_ADDR*/ 0;


    auto runtime_str = logging::Msg::dynamic(40 * 6);
    auto state_str   = logging::Msg::dynamic(40 * 6);
    vTaskGetRunTimeStats(runtime_str.as_chars());
    vTaskList(state_str.as_chars());

    // std::replace(runtime_str.data, runtime_str.data + strlen(runtime_str.as_chars()),
    //         '%', '&');
    std::replace(runtime_str.data, runtime_str.data + strlen(runtime_str.as_chars()),
            '\r', ' ');
    // for (size_t i = 0; i < strlen(runtime_str.as_chars()); i++) {
    //     if (runtime_str.data[i] == '%')
    //         runtime_str.data[i];
    // }


    auto stats = logging::Msg::dynamic(
            sizeof(stats_fmt) + 3 * 10 +
            strlen(runtime_str.as_chars()) + strlen(state_str.as_chars())
            );
    snprintf(stats.as_chars(), stats.size, stats_fmt,
            xPortGetFreeHeapSize(),
            xPortGetMinimumEverFreeHeapSize(),
            // free_stack,
            state_str.as_chars(),
            runtime_str.as_chars()
            );
    logging::log_blocking(stats);
    runtime_str.delete_if_owned();
    state_str.delete_if_owned();



    // auto msg = logging::Msg::dynamic(140 + 3 * 8);
    // snprintf(msg.as_chars(), msg.size,
    //         "================ STATS ====================\n"
    //         "=== MEMORY:\n"
    //         "free heap: %d\n"
    //         "minimum free heap: %d\n"
    //         // "free stack from here: %lu\n"
    //         "=== RUNTIME:\n",
    //         xPortGetFreeHeapSize(),
    //         xPortGetMinimumEverFreeHeapSize()
    //         // free_stack
    //         );
    // logging::log_blocking(msg);
    //
    // // runtime
    // msg = logging::Msg::dynamic(6 * 40);
    // vTaskGetRunTimeStats(msg.as_chars());
    // logging::log_blocking(msg);
    //
    // // states: 'B'locked, 'R'eady, 'D'eleted, 'S'uspended
    // // state | priority | stack (min. free) | num
    // logging::printf_blocking(20, "=== STATE:\n");
    // // logging::printf_blocking(60, " e[X]ecuting [B]locked [R]eady [S]uspended [D]eleted\n");
    // logging::printf_blocking(60, " Name * State * Priority * Stack * Num\n");
    // msg = logging::Msg::dynamic(6 * 40);
    // vTaskList(msg.as_chars());
    // logging::log_blocking(msg);
    //
    logging::printf_blocking(50, "===========================================\n");
}


void stats_task(void *) {
    auto last_start = xTaskGetTickCount();

    while(1) {
        logging::printf_blocking(3, "\n");
        print_system_statistics();
        logging::printf_blocking(3, "\n");
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(1000));
    }
}

void dummy_task(void *number_param) {
    int counter = 0;
    uintptr_t number = reinterpret_cast<uintptr_t>(number_param);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(300));

        logging::printf(100, "This is dummy task %d, counter = %d\n", number, counter++);
    }
}

void run() {
    logging::printf_blocking(100, "\n===========================================\n");
    logging::printf_blocking(100, "Initialising system...\n");

    /*** Prepare FreeRTOS tasks ***********************************************/

    // Most tasks are implemented as singletons with lazy-evaluation, i.e.
    // the object is constructed on first call to get(), so we need to
    // create these tasks here, before starting the scheduler.

    bool all_created = true;
    // all_created &= xTaskCreate(dummy_task, "Dummy 1",
    //         configMINIMAL_STACK_SIZE + 64, (void *) 1, 2, nullptr) == pdPASS;
    // all_created &= xTaskCreate(dummy_task, "Dummy 2",
    //         configMINIMAL_STACK_SIZE + 64, (void *) 2, 2, nullptr) == pdPASS;
    all_created &= xTaskCreate(stats_task, "Stats",
            configMINIMAL_STACK_SIZE *  2, nullptr, 1, nullptr) == pdPASS;
    configASSERT(all_created);

    /*** Print debug memory debug information *********************************/

    // this allows to optimize heap size that we assigned in FreeRTOSConfig.h
    // (more heap can be needed if anything is created dynamically later)
    size_t heap_size_remaining = xPortGetFreeHeapSize();
    logging::printf_blocking(60, "Remaining heap size = %u KB (%u B)\n",
            heap_size_remaining / (1 << 10), heap_size_remaining);

    /*** Start FreeRTOS scheduler *********************************************/

    logging::printf_blocking(100, "Starting scheduler...\n");

    // start RTOS event loop
    // IMPORTANT NOTE! this resets stack pointer, so all variables declared in
    // this scope will be overwritten - if needed, declare them globally or
    // allocate dynamically
    vTaskStartScheduler();

    // execution should never reach here, if it did, then something went wrong
    configASSERT(0);
}


