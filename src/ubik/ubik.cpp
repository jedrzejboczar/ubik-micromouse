#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "logging/logging.h"

void run();
extern "C" void extern_main(void) { run(); }

void dummy_task(void *) {

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        size_t heap_size_remaining = xPortGetFreeHeapSize();
        logging::printf(60, "Remaining heap size = %u KB (%u B)\n",
                heap_size_remaining / (1 << 10), heap_size_remaining);

        logging::printf_blocking(100, "Blocking printf...\n");

        {
#if 0
            logging::Msg msg;
            msg = logging::Msg::dynamic(60);
            snprintf(msg.as_chars(), msg.size, "Hello world!");
            logging::log(msg);
            msg = logging::Msg::dynamic(60);
            snprintf(msg.as_chars(), msg.size, "Hello world %d!", 10);
            logging::log(msg);
            msg = logging::Msg::dynamic(60);
            snprintf(msg.as_chars(), msg.size, "Hello world %d %d!", 10, 10);
            logging::log(msg);
            msg = logging::Msg::dynamic(60);
            snprintf(msg.as_chars(), msg.size, "Hello world %d %d %d!", 10, 10, 10);
            logging::log(msg);
#else
            logging::printf(100, "Hello world!\n");
            logging::printf(100, "Hello world %d!\n", 10);
            logging::printf(100, "Hello world %d %d!\n", 10, 10);
            logging::printf(100, "Hello world %d %d %d!\n", 10, 10, 10);
#endif
        }

    }
}

void run() {

    /*** Initialize debug routines for usage with FreeRTOS ********************/

    // TODO:
    // // set orange LED on until the scheduler is started
    // HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    // // set blue LED on until servos are initialised
    // HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);

    // DEBUG_FREERTOS_INIT();
    // D_PRINT("Hello world!\n");

    /*** Prepare FreeRTOS tasks ***********************************************/

    // Most tasks are implemented as singletons with lazy-evaluation, i.e.
    // the object is constructed on first call to get(), so we need to
    // create these tasks here, before starting the scheduler.

    bool task_created = xTaskCreate(dummy_task, "dummy",
            2 * configMINIMAL_STACK_SIZE, nullptr, 2, nullptr) == pdPASS;
    configASSERT(task_created);

    /*** Print debug memory debug information *********************************/

    // this allows to optimize heap size that we assigned in FreeRTOSConfig.h
    // (more heap can be needed if anything is created dynamically later)
    // volatile size_t heap_size_remaining = xPortGetFreeHeapSize();
    // uint8_t *buf = new uint8_t[50];
    // snprintf(reinterpret_cast<char *>(buf), 50,
    //         "Remaining heap size = %u KB (%u B)\n",
    //         heap_size_remaining / (1<<10), heap_size_remaining);
    // logging::log(logging::Buffer{buf, 50, true});

    /*** Start FreeRTOS scheduler *********************************************/

    // start RTOS event loop
    // IMPORTANT NOTE! this resets stack pointer, so all variables declared in this scope
    // will be overwritten - if needed, declare them globally or allocacte dynamically
    // TODO: HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    logging::printf_blocking(100, "Starting scheduler...\n");
    vTaskStartScheduler();

    // execution should never reach here, if it did, then something went wrong
    configASSERT(0);
}


