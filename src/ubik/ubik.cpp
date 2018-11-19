#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "timing.h"
#include "system_monitor.h"
#include "ubik/logging/logging.h"
#include "ubik/logging/stats.h"

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


void movement::Controller::delay(float dt) {
    vTaskDelay(pdMS_TO_TICKS(1e3f * dt));
}

void set_target_position_task(void *) {
    using constants::deg2rad;
    movement::Controller controller(100);

    float vel_lin = 0.50;
    float acc_lin = 0.40;
    float vel_ang = deg2rad(300);
    float acc_ang = deg2rad(400);

    while (1) {
        logging::printf(50, "Next cycle\n");

        using movement::Vec2;
        controller.move_arc({0,   deg2rad(45)   }, { 0, vel_ang }, { 0, acc_ang });
        controller.move_arc({0,   deg2rad(-90)  }, { 0, vel_ang }, { 0, acc_ang });
        controller.move_arc({0,   deg2rad(45)   }, { 0, vel_ang }, { 0, acc_ang });
        controller.move_arc({.20, 0             }, { vel_lin, 0 }, { acc_lin, 0 });
        controller.move_arc({0,   deg2rad(180)  }, { 0, vel_ang }, { 0, acc_ang });
        controller.move_arc({.20, 0             }, { vel_lin, 0 }, { acc_lin, 0 });
        controller.move_arc({0,   deg2rad(-180) }, { 0, vel_ang }, { 0, acc_ang });

        // spi::update_gpio_expander_pins(1 << 6, 1 << 7);
        vTaskDelay(pdMS_TO_TICKS(1500));
        // spi::update_gpio_expander_pins(1 << 7, 1 << 6);
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
    all_created &= xTaskCreate(system_monitor_task, "SysMonitor",
            configMINIMAL_STACK_SIZE * 2, nullptr, 3, nullptr) == pdPASS;
    all_created &= xTaskCreate(movement::regulator::regulation_task, "Regulator",
            configMINIMAL_STACK_SIZE * 2, nullptr, 4, nullptr) == pdPASS;
    all_created &= xTaskCreate(logging::stats_monitor_task, "Stats",
            configMINIMAL_STACK_SIZE * 2, reinterpret_cast<void *>(pdMS_TO_TICKS(10*1000)), 1, nullptr) == pdPASS;
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
