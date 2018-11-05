#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ubik/logging/logging.h"
#include "ubik/logging/stats.h"

void run();
extern "C" void extern_main(void) { run(); }

#include "timing.h"

void stats_task(void *) {
    auto last_start = xTaskGetTickCount();

    while(1) {
        logging::print_stats();
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(3000));
    }
}

void dummy_task(void *number_param) {
    int counter = 0;
    uintptr_t number = reinterpret_cast<uintptr_t>(number_param);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));

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
    all_created &= xTaskCreate(dummy_task, "Dummy 1",
            configMINIMAL_STACK_SIZE + 64, (void *) 1, 2, nullptr) == pdPASS;
    all_created &= xTaskCreate(dummy_task, "Dummy 2",
            configMINIMAL_STACK_SIZE + 64, (void *) 2, 2, nullptr) == pdPASS;
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


