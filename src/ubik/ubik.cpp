#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ubik/logging/logging.h"
#include "ubik/logging/stats.h"
#include "timing.h"

#include "movement/controller.h"


void run();
extern "C" void extern_main(void) { run(); }


extern "C" TIM_HandleTypeDef htim4;
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
    vTaskDelay(1500);

    using constants::deg2rad;
    movement::Controller controller(1000);
    movement::motors::set_enabled(true);

    float vel_lin = 0.50;
    float acc_lin = 0.40;
    float vel_ang = deg2rad(300);
    float acc_ang = deg2rad(400);

    while (1) {

        controller.move_turn(deg2rad(45), vel_ang, acc_ang);   controller.reset();
        controller.move_turn(deg2rad(-90), vel_ang, acc_ang);  controller.reset();
        controller.move_turn(deg2rad(45), vel_ang, acc_ang);   controller.reset();
        controller.move_line(.20, vel_lin, acc_lin);           controller.reset();
        controller.move_turn(deg2rad(180), vel_ang, acc_ang);  controller.reset();
        controller.move_line(.20, vel_lin, acc_lin);           controller.reset();
        controller.move_turn(deg2rad(-180), vel_ang, acc_ang); controller.reset();

    }


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
    movement::regulator::initialise(); // PID regulator

    /*** Prepare FreeRTOS tasks ***********************************************/

    // Most tasks are implemented as singletons with lazy-evaluation, i.e.
    // the object is constructed on first call to get(), so we need to
    // create these tasks here, before starting the scheduler.

    bool all_created = true;
    all_created &= xTaskCreate(set_target_position_task, "Setter",
            configMINIMAL_STACK_SIZE * 3, nullptr, 2, nullptr) == pdPASS;
    all_created &= xTaskCreate(movement::regulator::regulation_task, "Regulator",
            configMINIMAL_STACK_SIZE * 2, nullptr, 4, nullptr) == pdPASS;
    all_created &= xTaskCreate(logging::system_monitor_task, "SysMonitor",
            configMINIMAL_STACK_SIZE * 2, (void *) pdMS_TO_TICKS(5000), 1, nullptr) == pdPASS;
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
