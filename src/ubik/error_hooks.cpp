/*
 * Implementation of FreeRTOS hooks being used
 */

#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "logging/logging.h"

extern "C" {

// primitive delay that doesn't use interrupts
void dumb_delay(uint32_t ms);
// use red LED to indicate error
void blinkErrorLED(int n_times, int delay_ms);

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName ) {
    (void) xTask;
    (void) pcTaskName;

    taskDISABLE_INTERRUPTS();

    uint8_t buf[250];
    auto msg = logging::Msg::from_static(buf);
    snprintf(msg.as_chars(), msg.size,
            "[# ERROR #] FreeRTOS stack overflow, task = %s\n", pcTaskName);
    logging::log_blocking(msg);

    while (1) {
        blinkErrorLED(2, 200);
        dumb_delay(1000);
    }
}

void vApplicationMallocFailedHook( void ) {
    taskDISABLE_INTERRUPTS();

    uint8_t buf[250];
    auto msg = logging::Msg::from_static(buf);
    snprintf(msg.as_chars(), msg.size,
            "[# ERROR #] FreeRTOS malloc failed\n");
    logging::log_blocking(msg);

    vTaskSuspendAll();

    while (1) {
        blinkErrorLED(5, 200);
        dumb_delay(1000);
    }
}

void vApplicationConfigAssertFailedHook(const char *file, int line) {
    taskDISABLE_INTERRUPTS();

    uint8_t buf[250];
    auto msg = logging::Msg::from_static(buf);
    snprintf(msg.as_chars(), msg.size,
            "[# ERROR #] asserion failed at %s:%d\n", file, line);
    logging::log_blocking(msg);

    while (1) {
        blinkErrorLED(1, 200);
    }
}

// void HardFault_Handler(void) {
//     while (1) {
//         blinkErrorLED(1, 1000);
//     }
// }



// imlpementation of helper functions

void dumb_delay(uint32_t ms) {
    // assume that this variable is really the core frequency
    const uint32_t cycles_per_ms = SystemCoreClock / 1000;
    const uint32_t cycles_to_wait = cycles_per_ms * ms;

    /*
     * The loop below has been adjusted based on dissassembly results.
     * The calculations above take some cycles too, but were not included.
     * Code has been setup in such way that the main loop looks exactly the
     * same for both -Og and -Os optiomization levels (checked by trial and error).
     *
     * Cortex-M4 cycle counts:
     * http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0439b/CHDDIGAC.html
     *
     * Loop code (branch destinations replaced with labels <1> and <2>):
     *   <1>  cmp    r3, r0     # 1 cycle
     *        bcs.n  <2>        # 1 cycle (or 2-4 when branch is taken)
     *        nop               # 1 cycle
     *        adds   r3, #1     # 1 cycle
     *        b.n    <1>        # 2-4 cycles (or 1 when taken)
     *   <2>  bx     lr         # returns from the function
     * This gives us something like 6 cycles per loop (or up to 8).
     *
     * These 1-3 cycles are for pipeline refill.
     * Citing documentation:
     *   "The number of cycles required for a pipeline refill.
     *    This ranges from 1 to 3 depending on the alignment
     *    and width of the target instruction, and whether
     *    the processor manages to speculate the address early."
     *
     * We'll assume 6 cycles as we hope that this will be minimal in a loop.
     */
    const uint32_t clocks_per_loop = 6;
    const uint32_t loop_steps = cycles_to_wait / clocks_per_loop;

    // wait that many clock cycles
    for (uint32_t i = 0; i < loop_steps; i++) {
        __NOP();
    }
}

void blinkErrorLED(int n_times, int delay_ms) {
    for (int i = 0; i < n_times; i++) {
        // HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
        dumb_delay(delay_ms);
        // HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
        dumb_delay(delay_ms);
    }
}


} // extern "C"
