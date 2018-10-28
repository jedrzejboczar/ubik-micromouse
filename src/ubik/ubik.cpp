#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "supervision.h"

void run();
extern "C" void extern_main(void) { run(); }

// TODO: D_PRINT
#define D_PRINT(...) (void) 0;
#define DEBUG_FREERTOS_INIT() (void) 0;


class MainTask: public supervision::SupervisorTask {
public:
    static MainTask& get() {
        static MainTask instance;
        return instance;
    }
private:
    MainTask(): supervision::SupervisorTask("Main", 256) { }

    /*** This is the main task that controls system as a whole ****************/
    virtual void run() override {
        start_all();

        vTaskDelay(portMAX_DELAY);
    }
};

#include "drivers/uart.h"
#include "drivers/freertos_driver.h"
extern UART_HandleTypeDef huart1;

void run() {

    /*** Initialize debug routines for usage with FreeRTOS ********************/

    // TODO:
    // // set orange LED on until the scheduler is started
    // HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    // // set blue LED on until servos are initialised
    // HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);

    DEBUG_FREERTOS_INIT();
    D_PRINT("Hello world!\n");

    /*** Prepare FreeRTOS tasks ***********************************************/

    // Most tasks are implemented as singletons with lazy-evaluation, i.e.
    // the object is constructed on first call to get(), so we need to
    // create these tasks here, before starting the scheduler.
    MainTask::get();

    UART uart(&huart1);
    FreeRTOSDriverTask task("uart", 3, 256, 5, 10);
    DeviceDriver<FreeRTOSDriverTask, UART> uart_driver(uart, task);

    // configASSERT(xTaskCreate(dummyServosTask, "dummy",
    //             configMINIMAL_STACK_SIZE, nullptr, 2, nullptr));

    /*** Print debug memory debug information *********************************/

    // this allows to optimize heap size that we assigned in FreeRTOSConfig.h
    // (more heap can be needed if anything is created dynamically later)
    volatile size_t heap_size_remaining = xPortGetFreeHeapSize();
    D_PRINT("Remaining heap size = %u KB (%u B)\n",
            heap_size_remaining / (1<<10), heap_size_remaining);

    /*** Start FreeRTOS scheduler *********************************************/

    // start RTOS event loop
    // IMPORTANT NOTE! this resets stack pointer, so all variables declared in this scope
    // will be overwritten - if needed, declare them globally or allocacte dynamically
    // TODO: HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    vTaskStartScheduler();

    // execution should never reach here, if it did, then something went wrong
    configASSERT(0);
}


