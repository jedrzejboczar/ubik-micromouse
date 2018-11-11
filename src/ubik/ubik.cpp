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
        logging::printf(50, "=== Printing stats took %d us ===\n", cycles_counter::get_us());
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(5000));
    }
}

#include "ubik/movement/pid.h"
#include "ubik/movement/motor_control.h"
#include "ubik/movement/as5045.h"
#include "ubik/logging/print_bits.h"
#include "ubik/movement/spi_devices.h"


// PID vel_pid;

extern "C" TIM_HandleTypeDef htim4;
spi::EncoderReadings current;
volatile int angle_left = 0;
volatile int angle_right = 0;

extern "C" void callback_timer_period_elapsed(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim4.Instance) {
        // read encoders
        spi::EncoderReadings readings = spi::read_encoders();
        if (readings.valid && readings.left.is_ok() && readings.right.is_ok()) {
            current = readings;
            angle_left = readings.left.angle;
            angle_right = readings.right.angle;
        }

        // get next PID output
        // set motors pulse
    }
}

extern "C" TIM_HandleTypeDef htim2;
extern "C" TIM_HandleTypeDef htim3;

void angle_printing_task(void *) {
    auto last_start = xTaskGetTickCount();
    while(1) {
        // logging::printf(200, "L: ok=%d ang=%5d R: ok=%d ang=%5d\n",
        //         current.left.is_ok(), current.left.angle, current.right.is_ok(), current.right.angle);
        logging::printf(200, "L %5d R %5d\n",
                current.left.angle, current.right.angle);
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(2));
    }
}

uint16_t left_angles[2000];

void mover_task(void *) {
    auto last_start = xTaskGetTickCount();


    spi::initialise();
    movement::driver::initialise();

    float f = 1000;
    HAL_TIM_Base_DeInit(&htim4);
	htim4.Init.Prescaler = 72 - 1; // 72MHz -> 1MHz
	htim4.Init.Period = ((uint32_t) 1000000 / f) - 1;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
		configASSERT(0);
	HAL_TIM_Base_Start_IT(&htim4);



    movement::driver::set_direction(movement::FORWARDS, movement::FORWARDS);
    movement::driver::set_enabled(true);
    logging::printf_blocking(50, "max_pulse = %d\n", movement::driver::max_pulse());
    float max_ratio = .3;
    int pulse = 0;

    for (int i = 0; i < 1000; i++) {
        pulse = i/1000.0f * max_ratio * movement::driver::max_pulse();
        movement::driver::set_pulse(pulse, pulse);
        if (i % 10 == 0)
            logging::printf_blocking(50, "pulse = %d\n", pulse);
        // vTaskDelayUntil(&last_start, pdMS_TO_TICKS(3));
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    for (int i = 1000; i > 0; i--) {
        pulse = i/1000.0f * max_ratio * movement::driver::max_pulse();
        movement::driver::set_pulse(pulse, pulse);
        if (i % 10 == 0)
            logging::printf_blocking(50, "pulse = %d\n", pulse);
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    movement::driver::set_enabled(false);


    while (1) {

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#include "movement/regulator.h"

void run() {
    logging::printf_blocking(100, "\n===========================================\n");
    logging::printf_blocking(100, "Initialising system...\n");

    /*** Prepare FreeRTOS tasks ***********************************************/

    // Most tasks are implemented as singletons with lazy-evaluation, i.e.
    // the object is constructed on first call to get(), so we need to
    // create these tasks here, before starting the scheduler.

    bool all_created = true;
    // all_created &= xTaskCreate(angle_printing_task, "AngPrint",
    //         configMINIMAL_STACK_SIZE * 2, nullptr, 1, nullptr) == pdPASS;
    // all_created &= xTaskCreate(mover_task, "Mover",
    //         configMINIMAL_STACK_SIZE * 2, nullptr, 3, nullptr) == pdPASS;
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
