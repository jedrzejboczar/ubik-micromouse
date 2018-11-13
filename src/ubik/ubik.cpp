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


void monitor(void *) {
    auto check_button = []() {
        return HAL_GPIO_ReadPin(SW_Start_GPIO_Port, SW_Start_Pin) == GPIO_PIN_SET;
    };

    bool regulation_on = false;
    int min_button_delay_ms = 500; // delay between subsequent button presses

    auto last_start = xTaskGetTickCount();
    int last_button_press = std::numeric_limits<int>::min();
    while (1) {
        // when the button has not been used for at least 1 second, to avoid double-press
        if (xTaskGetTickCount() - last_button_press > pdMS_TO_TICKS(min_button_delay_ms)) {
            // sample the button for <100 ms
            int button_on_count = 0;
            for (int i = 0; i < 10; i++) {
                if (check_button())
                    button_on_count++;
                vTaskDelay(8);
            }

            // register button press when there were 70-100 % positives
            if (button_on_count > 6) {
                // toggle regulation state on button press
                regulation_on = !regulation_on;
                logging::printf(100, "Setting regulation state: %s\n", regulation_on ? "ON" : "OFF");
                movement::regulator::set_enabled(regulation_on);

                // reset button timer
                last_button_press = xTaskGetTickCount();
            }
        }

        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(100));
    }
}

void set_target_position_task(void *) {
    using constants::deg2rad;
    movement::Controller controller(1000);

    float vel_lin = 0.50;
    float acc_lin = 0.40;
    float vel_ang = deg2rad(300);
    float acc_ang = deg2rad(400);

    while (1) {
        logging::printf(50, "Next cycle\n");
        controller.move_turn(deg2rad(45), vel_ang, acc_ang);   controller.reset();
        controller.move_turn(deg2rad(-90), vel_ang, acc_ang);  controller.reset();
        controller.move_turn(deg2rad(45), vel_ang, acc_ang);   controller.reset();
        controller.move_line(.20, vel_lin, acc_lin);           controller.reset();
        controller.move_turn(deg2rad(180), vel_ang, acc_ang);  controller.reset();
        controller.move_line(.20, vel_lin, acc_lin);           controller.reset();
        controller.move_turn(deg2rad(-180), vel_ang, acc_ang); controller.reset();
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
    all_created &= xTaskCreate(monitor, "Monitor",
            configMINIMAL_STACK_SIZE * 2, (void *) nullptr, 3, nullptr) == pdPASS;
    all_created &= xTaskCreate(movement::regulator::regulation_task, "Regulator",
            configMINIMAL_STACK_SIZE * 2, nullptr, 4, nullptr) == pdPASS;
    all_created &= xTaskCreate(logging::stats_monitor_task, "Stats",
            configMINIMAL_STACK_SIZE * 2, (void *) pdMS_TO_TICKS(10*1000), 1, nullptr) == pdPASS;
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
