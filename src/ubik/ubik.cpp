#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "logging/logging.h"

void run();
extern "C" void extern_main(void) { run(); }

void* operator new(size_t size) {
    void *memory = pvPortMalloc(size);
#ifdef	__EXCEPTIONS
    if (memory == 0) // did pvPortMalloc succeed?
        throw std::bad_alloc(); // ANSI/ISO compliant behavior
#endif
    return memory;
}
void operator delete(void *memory) noexcept
{
    vPortFree(memory);
}
void operator delete(void *memory, size_t) { // ? required by C++14 ?
    vPortFree(memory);
}

void dummy_task(void *) {
    // we need to set the bluetooth pin "Key" to HIGH to enable normal mode
    HAL_GPIO_WritePin(BT_Key_GPIO_Port, BT_Key_Pin, GPIO_PIN_SET);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        volatile size_t heap_size_remaining = xPortGetFreeHeapSize();
        uint8_t *buf = new uint8_t[50];
        snprintf(reinterpret_cast<char *>(buf), 50,
                "Remaining heap size = %u KB (%u B)\n",
                heap_size_remaining / (1<<10), heap_size_remaining);
        logging::log(logging::Buffer{buf, 50, true});

        // vTaskDelay(pdMS_TO_TICKS(1000));
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

    configASSERT(xTaskCreate(dummy_task, "dummy",
                configMINIMAL_STACK_SIZE, nullptr, 2, nullptr));

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
    vTaskStartScheduler();

    // execution should never reach here, if it did, then something went wrong
    configASSERT(0);
}


