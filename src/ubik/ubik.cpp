#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "timing.h"
#include "system_monitor.h"
#include "ubik/logging/logging.h"
#include "ubik/logging/stats.h"

#include "movement/regulator.h"
#include "movement/controller.h"
#include "distance_sensors.h"


void run();
extern "C" void extern_main(void) { run(); }

// extern "C" TIM_HandleTypeDef htim4;
// extern "C" void callback_timer_period_elapsed(TIM_HandleTypeDef *htim) {
//     if (htim->Instance == htim4.Instance) {
//         // // read encoders
//         // spi::EncoderReadings readings = spi::read_encoders();
//         // if (readings.valid && readings.left.is_ok() && readings.right.is_ok()) {
//         //     current = readings;
//         //     angle_left = readings.left.angle;
//         //     angle_right = readings.right.angle;
//         // }
//
//         // get next PID output
//         // set motors pulse
//     }
// }


void distance_sensors_task(void *) {
    vTaskDelay(pdMS_TO_TICKS(300));

    while(1) {
#if 0
        auto readings = distance_sensors::read(spi::gpio::DISTANCE_SENSORS_ALL());
#else
        uint8_t sensors = spi::gpio::DISTANCE_SENSORS[0] | spi::gpio::DISTANCE_SENSORS[1]
            | spi::gpio::DISTANCE_SENSORS[4] | spi::gpio::DISTANCE_SENSORS[5];
        auto readings = distance_sensors::read( sensors);
#endif

        logging::printf(100, "%4d %4d %4d %4d %4d %4d\n\n",
                readings.sensor[0],
                readings.sensor[1],
                readings.sensor[2],
                readings.sensor[3],
                readings.sensor[4],
                readings.sensor[5]);

        vTaskDelay(50);
    }

    vTaskDelay(portMAX_DELAY);
}


void movement::Controller::delay(float dt) {
    vTaskDelay(pdMS_TO_TICKS(1e3f * dt));
}

void set_target_position_task(void *) {
    using constants::deg2rad;
    movement::Controller c(100);

    float vel_lin = 0.50;
    float acc_lin = 0.40;
    float vel_ang = deg2rad(300);
    float acc_ang = deg2rad(400);

    while (1) {
        logging::printf(50, "Next cycle\n");

        using movement::Vec2;

        // move on an equilateral traingle with a=10cm
        float a = 0.15;
        float R = 0.577f * a;

        c.move_line(a, vel_lin, acc_lin);
        c.move_rotate(deg2rad(120), vel_ang, acc_ang);
        c.move_line(a, vel_lin, acc_lin, 0);
        // c.move_rotate(deg2rad(120), vel_ang, acc_ang);
        // c.move_line(a, vel_lin, acc_lin, 0);
        // c.move_rotate(deg2rad(120), vel_ang, acc_ang);

        // move on the circle circumscribing the triangle
        c.move_rotate(deg2rad(60), vel_ang, acc_ang);
        c.move_arc({deg2rad(120) * R, deg2rad(120)}, vel_lin, acc_lin);
        // turn back
        c.move_rotate(deg2rad(-180 - 120), vel_ang, acc_ang);

        spi::gpio::update_pins(spi::gpio::LED_RED, spi::gpio::LED_BLUE);
        vTaskDelay(pdMS_TO_TICKS(1500));
        spi::gpio::update_pins(spi::gpio::LED_BLUE, spi::gpio::LED_RED);
    }
}

void run() {
    logging::printf_blocking(100, "\n===========================================\n");
    logging::printf_blocking(100, "Initialising system...\n");

    /*** Initialize modules ***************************************************/

    spi::initialise(); // encoders & gpio expander
    distance_sensors::initialise(); // encoders & gpio expander
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
            configMINIMAL_STACK_SIZE * 2, nullptr, 4, nullptr) == pdPASS;
    all_created &= xTaskCreate(movement::regulator::regulation_task, "Regulator",
            configMINIMAL_STACK_SIZE * 2, nullptr, 5, nullptr) == pdPASS;
    all_created &= xTaskCreate(logging::stats_monitor_task, "Stats",
            configMINIMAL_STACK_SIZE * 2, reinterpret_cast<void *>(pdMS_TO_TICKS(10*1000)), 1, nullptr) == pdPASS;
    all_created &= xTaskCreate(distance_sensors_task, "Distance",
            configMINIMAL_STACK_SIZE * 2, nullptr, 3, nullptr) == pdPASS;
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
