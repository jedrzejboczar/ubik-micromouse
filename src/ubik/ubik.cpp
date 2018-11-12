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
        cycles_counter::reset();
        cycles_counter::start();
        logging::print_stats();
        cycles_counter::stop();
        logging::printf(70, "=== Printing stats took %d us ===\n", cycles_counter::get_us());
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(7.5e3));
    }
}

#include "ubik/movement/motor_control.h"
// #include "ubik/movement/as5045.h"
// #include "ubik/logging/print_bits.h"
#include "ubik/movement/spi_devices.h"
#include "movement/regulator.h"
#include "movement/controller.h"


extern "C" TIM_HandleTypeDef htim4;
spi::EncoderReadings current;
volatile int angle_left = 0;
volatile int angle_right = 0;

extern "C" void callback_timer_period_elapsed(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim4.Instance) {
        // // read encoders
        // spi::EncoderReadings readings = spi::read_encoders();
        // if (readings.valid && readings.left.is_ok() && readings.right.is_ok()) {
        //     current = readings;
        //     angle_left = readings.left.angle;
        //     angle_right = readings.right.angle;
        // }

        // get next PID output
        // set motors pulse
    }
}

void set_target_position_task(void *) {
    vTaskDelay(1000);

    movement::Controller controller(500);

    movement::motors::set_enabled(true);
    controller.move_line(.10, 0.25, 0.3, 0.05);
    controller.move_line(.20, 0.05, 0.3);


    // movement::motors::set_enabled(true);
    // movement::regulator::set_regulation_target(0.1, 0);
    // vTaskDelay(2000);
    // // movement::motors::set_enabled(false);

    // movement::motors::set_enabled(true);
    // int imax = 2500;
    // for (int i = 0; i < 2500; i++) {
    //     // logging::printf(100, "setting trans = %f\n", 0.001 * i);
    //     // logging::printf(100, "setting trans = %d / 1000\n", 1 * i);
    //     movement::regulator::set_regulation_target(0.10 * i / imax, 0);
    //     vTaskDelay(2);
    // }
    // // movement::motors::set_enabled(false);

    while(1) {
        vTaskDelay(1000);
    }
}

void run() {
    logging::printf_blocking(100, "\n===========================================\n");
    logging::printf_blocking(100, "Initialising system...\n");

    /*** Initialize modules ***************************************************/

    spi::initialise(); // encoders & gpio expander
    movement::motors::initialise(); // motor control

    /*** Prepare FreeRTOS tasks ***********************************************/

    // Most tasks are implemented as singletons with lazy-evaluation, i.e.
    // the object is constructed on first call to get(), so we need to
    // create these tasks here, before starting the scheduler.

    bool all_created = true;
    // all_created &= xTaskCreate(angle_printing_task, "AngPrint",
    //         configMINIMAL_STACK_SIZE * 2, nullptr, 1, nullptr) == pdPASS;
    // all_created &= xTaskCreate(mover_task, "Mover",
    //         configMINIMAL_STACK_SIZE * 2, nullptr, 3, nullptr) == pdPASS;
    all_created &= xTaskCreate(set_target_position_task, "Setter",
            configMINIMAL_STACK_SIZE * 2, nullptr, 2, nullptr) == pdPASS;
    all_created &= xTaskCreate(movement::regulator::regulation_task, "Regulator",
            configMINIMAL_STACK_SIZE * 2, nullptr, 3, nullptr) == pdPASS;
    // all_created &= xTaskCreate(stats_task, "Stats",
    //         configMINIMAL_STACK_SIZE * 2, nullptr, 2, nullptr) == pdPASS;
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
