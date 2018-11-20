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

void dummy_delay_us(uint32_t micros) {
    cycles_counter::reset();
    cycles_counter::start();
    while (cycles_counter::get_us() < micros) {  }
    cycles_counter::stop();
}

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


/*
 * ADC1, fclk = 9MHz
 * single conversion: Ts = (12.5 + 28.5) * 1/fclk ~= 4.556 us
 * 6 conversions: T6 = 27.333 us
 * DMA: periph(no-inc)->memory(inc), not-circular, half-word
 *
 * Numeration:
 * - in schematics:
 *                   Front
 *          _______________________
 *  Left   /      o         o      \   Right
 *        /      /           \      \
 *       /      /             \      \
 *      |       DS4         DS3      |
 *      |  o                      o  |
 *      |  |                      |  |
 *      |   DS5               DS2    |
 *      |                            |
 *      | o-- DS6            DS1 --o |
 *      |                            |
 *
 * - sensor to ADC1 channel mapping: DS(N) -> CH(N-1)
 * - in code we use 0-indexing: DS(N) -> DS(N-1)
 * Conversion regular channels order: 0,1,2,3,4,5
 *   (instead of the old_ubik's 0,2,4,1,3,5 that was using discontinuous conversions)
 *
 * Gpio expander used for turning LEDs on/off:
 *    LEDS are connected in different order than in pcb/plots/ubik.pdf TODO: update this pdf
 *    they are actually connected in the order: GPIOEX 0-5 --> DS 4,3,2,1,5,6
 *
 */
extern "C" ADC_HandleTypeDef hadc1;

struct DSReading {
    uint16_t vals[6];
};
DSReading measurements[200] = {0};
int ds_index = 0;

void distance_sensors(void *) {
    uint16_t readings[6] = {0};
    bool ok;
    ADC_HandleTypeDef &hadc = hadc1;

    // calibrate ADC
    ok = HAL_ADCEx_Calibration_Start(&hadc) == HAL_OK;
    configASSERT(ok);

    vTaskDelay(pdMS_TO_TICKS(1000));


    {
        uint8_t led = spi::gpio::DISTANCE_SENSORS[0];
        int time_us = 5;

        auto read_next = [](int us) {
            HAL_ADC_Start_DMA(&hadc, reinterpret_cast<uint32_t *>(measurements[ds_index++].vals), 6);
            dummy_delay_us(us);
            HAL_ADC_Stop_DMA(&hadc);
        };

        // initial read
        read_next(time_us);

        // turn on LED
        spi::gpio::update_pins(led, 0);

        // wait some time
        for (int i = 0; i < 20; i++) {
            read_next(time_us);
        }

        // turn off LED
        spi::gpio::update_pins(0, led);

        // wait some time
        for (int i = 0; i < 50; i++) {
            read_next(time_us);
        }

        // print CSV
        for (int i = 0; i < ds_index; i++) {
            uint16_t *buf = measurements[i].vals;
            logging::printf(100, "%d,%d,%d,%d,%d,%d\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
            vTaskDelay(5);
        }
    }


    vTaskDelay(5000000);
    while(1) {
        for (int i = 0; i < n_elements(spi::gpio::DISTANCE_SENSORS); i++) {
            uint8_t sensor = spi::gpio::DISTANCE_SENSORS[i];

            // read the ADC
            HAL_ADC_Start_DMA(&hadc, reinterpret_cast<uint32_t *>(readings), 6);
            dummy_delay_us(10);
            logging::printf(100, "%4d %4d %4d %4d %4d %4d\n",
                    readings[0], readings[1], readings[2], readings[3], readings[4], readings[5]);

            // turn on LED
            spi::gpio::update_pins(sensor, 0);
            dummy_delay_us(150);

            // measure
            HAL_ADC_Start_DMA(&hadc, reinterpret_cast<uint32_t *>(readings), 6);
            dummy_delay_us(10);

            // turn off LED
            spi::gpio::update_pins(0, sensor);


            logging::printf(100, "%4d %4d %4d %4d %4d %4d\n\n",
                    readings[0], readings[1], readings[2], readings[3], readings[4], readings[5]);

            vTaskDelay(10);
        }

        vTaskDelay(1000);
    }

    vTaskDelay(portMAX_DELAY);
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

        // spi::gpio::update_pins(spi::gpio::LED_RED, spi::gpio::LED_BLUE);
        vTaskDelay(pdMS_TO_TICKS(1500));
        // spi::gpio::update_pins(spi::gpio::LED_BLUE, spi::gpio::LED_RED);
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
    all_created &= xTaskCreate(distance_sensors, "Distance",
            configMINIMAL_STACK_SIZE * 2, nullptr, 1, nullptr) == pdPASS;
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
