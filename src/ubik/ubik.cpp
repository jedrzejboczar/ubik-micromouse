#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ubik/common/timing.h"
#include "ubik/logging/logging.h"
#include "ubik/logging/stats.h"
#include "system_monitor.h"

#include "localization/odometry.h"
#include "movement/regulator.h"
#include "movement/controller.h"
#include "common/distance_sensors.h"
#include "maze/maze.h"
#include "maze/maze_definitions.h"


void run();
extern "C" void extern_main(void) { run(); }


void set_target_position_task(void *) {
    vTaskDelay(pdMS_TO_TICKS(500));

    using constants::deg2rad;
    namespace ctrl = movement::controller;

    ctrl::set_frequency(100);

    // select velocities
    system_monitor::lock_button();
    float vel_lin = system_monitor::select_with_wheels(0.3, {0.0, 3.0}, 3.0, "vel_lin =");
    float acc_lin = system_monitor::select_with_wheels(0.3, {0.0, 3.0}, 3.0, "acc_lin =");
    float vel_ang = system_monitor::select_with_wheels(deg2rad(360), {0.0, 5 * deg2rad(360)}, 5 * deg2rad(360), "vel_ang =");
    float acc_ang = system_monitor::select_with_wheels(deg2rad(360), {0.0, 5 * deg2rad(360)}, 5 * deg2rad(360), "acc_ang =");
    system_monitor::unlock_button();

    int choice = 0;
    while (1) {
        logging::printf(50, "Next cycle\n");

        enum {
            TRIANGLE = 0, EIGHT, CIRCLE,
            COUNT
        };

        spi::gpio::update_pins(spi::gpio::LED_RED, spi::gpio::LED_BLUE);
        system_monitor::lock_button();
        choice = system_monitor::select_with_wheels_int(choice, {0, COUNT-1}, 2 * COUNT, "choice =");
        system_monitor::unlock_button();
        spi::gpio::update_pins(spi::gpio::LED_BLUE, spi::gpio::LED_RED);

        switch (choice) {
            case TRIANGLE:
                {
                    // move on an equilateral traingle with a=10cm
                    float a = 0.15;
                    float R = 0.577f * a;

                    ctrl::move_line(a, vel_lin, acc_lin);
                    ctrl::move_rotate(deg2rad(120), vel_ang, acc_ang);
                    ctrl::move_line(a, vel_lin, acc_lin, 0);
                    // ctrl::move_rotate(deg2rad(120), vel_ang, acc_ang);
                    // ctrl::move_line(a, vel_lin, acc_lin, 0);
                    // ctrl::move_rotate(deg2rad(120), vel_ang, acc_ang);

                    // move on the circle circumscribing the triangle
                    ctrl::move_rotate(deg2rad(60), vel_ang, acc_ang);
                    ctrl::move_arc({deg2rad(120) * R, deg2rad(120)}, vel_lin, acc_lin);
                    // turn back
                    ctrl::move_rotate(deg2rad(-180 - 120), vel_ang, acc_ang);
                    break;
                }
            case EIGHT:
                {
                    // move in an eight
                    float R = 0.08;
                    float angle = deg2rad(270);
                    float arc_len = constants::arc_length(angle, R);
                    int N = 2;
                    float vel_final = 0.3f * vel_lin;
                    // get initial speed
                    ctrl::move_line(R, vel_lin, acc_lin, vel_final);
                    for (int i = 0; i < N; i++) {
                        ctrl::move_arc({arc_len, -angle}, vel_lin, acc_lin, vel_final);
                        ctrl::move_line(2 * R, vel_lin, acc_lin, vel_final);
                        ctrl::move_arc({arc_len, angle}, vel_lin, acc_lin, vel_final);
                        if (i < N - 1)
                            ctrl::move_line(2 * R, vel_lin, acc_lin, vel_final);
                        else
                            ctrl::move_line(R, vel_lin, acc_lin, 0);
                    }
                    break;
                }
            case CIRCLE:
                {
                    // move in circle
                    float R = 0.15;
                    float angle = deg2rad(360);
                    float arc_len = constants::arc_length(angle, R);
                    ctrl::move_arc({2*arc_len, 2*angle}, vel_lin, acc_lin);
                    ctrl::move_arc({-2*arc_len, -2*angle}, vel_lin, acc_lin); // unwind the cables xD
                    break;
                }

        }

    }
}


void distance_sensors_task(void *) {
    vTaskDelay(pdMS_TO_TICKS(300));

    while (1) {
#if 1
        auto readings = distance_sensors::read(spi::gpio::DISTANCE_SENSORS_ALL());
#else
        uint8_t sensors = spi::gpio::DISTANCE_SENSORS[0] | spi::gpio::DISTANCE_SENSORS[1]
            | spi::gpio::DISTANCE_SENSORS[4] | spi::gpio::DISTANCE_SENSORS[5];
        auto readings = distance_sensors::read( sensors);
#endif

        // won't always work as expected, but basically, print when not in select_with_wheels()
        if (! system_monitor::is_button_locked() || system_monitor::get_regulation_state() == true) {
            logging::printf(100, "%4d %4d %4d %4d %4d %4d\n\n",
                    readings.sensor[0],
                    readings.sensor[1],
                    readings.sensor[2],
                    readings.sensor[3],
                    readings.sensor[4],
                    readings.sensor[5]);
        }

        vTaskDelay(50);
    }

    vTaskDelay(portMAX_DELAY);
}

float cos_tailor_1q(float x) {
    return 1
        - x*x / 2
        + x*x*x*x / 24
        - x*x*x*x*x*x / 720
        + x*x*x*x*x*x*x*x / 40320;
}

float cos_tailor(float x) {
    constexpr float two_pi = 2 * PI;
    constexpr float half_pi = PI/2;
    x = std::fmod(x, two_pi);
    if (x < 0)
        x *= -1;
    int quadrant = static_cast<int>(x / half_pi);
    switch (quadrant) {
        case 0: return cos_tailor_1q(x);
        case 1: return -cos_tailor_1q(PI - x);
        case 2: return -cos_tailor_1q(x - PI);
        case 3: return cos_tailor_1q(two_pi - x);
        default: return 0;
    }
}

void test_sine_cosine() {
#if 1
    float trans = 0.001;
    for (float theta = 0.0; theta < 2*PI; theta += PI/400) {
        cycles_counter::reset();
        cycles_counter::start();
        float delta_x = trans * std::cos(theta);
        cycles_counter::stop();
        logging::printf(100, "%12.9f = %8.6f * std::cos(%8.6f), cycles = %d\n",
                delta_x, trans, theta, cycles_counter::get());
        vTaskDelay(10);
    }
#else
    float trans = 0.001;
    for (float theta = 0.0; theta < 2*PI; theta += PI/400) {
        cycles_counter::reset();
        cycles_counter::start();
        float delta_x = trans * cos_tailor(theta);
        cycles_counter::stop();
        logging::printf(100, "%12.9f = %8.6f * cos(%8.6f), cycles = %d\n",
                delta_x, trans, theta, cycles_counter::get());
        vTaskDelay(10);
    }
#endif
}

void maze_task(void *) {
    namespace ctrl = movement::controller;
    using namespace system_monitor;

    vTaskDelay(300);
    test_sine_cosine();
    vTaskDelay(portMAX_DELAY);

    // set controller frequency
    ctrl::set_frequency(100);

    // show distance sensors readings until button pressed
    lock_button();
    while (! wait_for_button_press(pdMS_TO_TICKS(100))) {
        auto readings = distance_sensors::read(spi::gpio::DISTANCE_SENSORS_ALL());
        logging::printf(100, "[sensors] %4d %4d %4d %4d %4d %4d\n",
                readings.sensor[0],
                readings.sensor[1],
                readings.sensor[2],
                readings.sensor[3],
                readings.sensor[4],
                readings.sensor[5]);
    }
    unlock_button();


    constexpr int MAX_MAZE_SIZE = 16;

    vTaskDelay(10);
    lock_button();
    const int maze_size = select_with_wheels_int(4, {1, MAX_MAZE_SIZE}, 2*MAX_MAZE_SIZE, "[maze] select size =");

    maze::Cell cells[maze_size * maze_size];
    StaticStack<maze::Position, MAX_MAZE_SIZE * MAX_MAZE_SIZE> maze_stack;
    maze::Maze maze(maze_size, maze_size, cells, maze_stack, maze::START_POSITION);

    logging::printf(80, "[maze] Size of maze: %d (size of data structures: %d)\n",
            maze_size, maze_size * maze_size * sizeof(maze::Cell) + sizeof(maze_stack) + sizeof(maze));
    unlock_button();

    while (1) {
        lock_button();
        spi::gpio::update_pins(spi::gpio::LED_BLUE, 0);
        // auto goal_pos = maze::TargetPosition(maze_size-1, maze_size-1);
        auto goal_pos = maze::TargetPosition(
                select_with_wheels_int(maze_size/2, {0, maze_size-1}, 2*maze_size, "[maze] target.x ="),
                select_with_wheels_int(maze_size/2, {0, maze_size-1}, 2*maze_size, "[maze] target.y =")
                );
        logging::printf(100, "[maze] Moving from (%d, %d) to (%.1f, %.1f)\n[maze] Start?\n",
                maze.position().x, maze.position().y, double(goal_pos.x), double(goal_pos.y));
        spi::gpio::update_pins(0, spi::gpio::LED_BLUE);
        wait_for_button_press(portMAX_DELAY);
        unlock_button();

        vTaskDelay(pdMS_TO_TICKS(2000));
        bool success = maze.go_to(goal_pos);
        logging::printf(100, "[maze] %s Current position (%d, %d)\n",
                success ? "Finished successfully." : "Could not finish the maze!",
                maze.position().x, maze.position().y);

        logging::printf(100, "[maze] Moving from (%d, %d) to (%.1f, %.1f)\n",
                maze.position().x, maze.position().y, double(goal_pos.x), double(goal_pos.y));
        if (success)
            success = maze.go_to(maze::START_POSITION);
        logging::printf(100, "[maze] %s Current position (%d, %d)\n",
                success ? "Finished successfully." : "Could not finish the maze!",
                maze.position().x, maze.position().y);
    }
}


void run() {
    logging::printf_blocking(100, "\n===========================================\n");
    logging::printf_blocking(100, "Initialising system...\n");

    /*** Initialize modules ***************************************************/
    // this mainly initializes all the peripheral devices
    // and allocates all the required queues and semaphores
    // TODO: create some tree-like hierarchy of initializations, as this gets unwieldy

    spi::initialise(); // encoders & gpio expander
    distance_sensors::initialise(); // distance sensors' ADC
    system_monitor::initialise(); // battery and regulation control
    localization::initialise(); // encoders odomoetry
    movement::motors::initialise(); // motor control
    movement::regulator::initialise(); // PID regulator

    /*** Create FreeRTOS tasks ************************************************/

    bool all_created = true;
    // auto create_task = [&all_created] (auto ...args) {
    //     auto result = xTaskCreate(args...);
    //     all_created = all_created && result == pdPASS;
    // };
    auto create_task = [&all_created]
        (auto name, auto priority, auto func, auto stack_size, auto param)
        {
            auto result = xTaskCreate(func, name, stack_size, param, priority, nullptr);
            all_created = all_created && result == pdPASS;
        };

    create_task("SysMonitor", 4, system_monitor::system_monitor_task,
            configMINIMAL_STACK_SIZE * 2, nullptr);
    create_task("Regulator", 5, movement::regulator::regulation_task,
            configMINIMAL_STACK_SIZE * 2, nullptr);
    // create_task("Stats", 1, logging::stats_monitor_task,
    //         configMINIMAL_STACK_SIZE * 2, reinterpret_cast<void *>(pdMS_TO_TICKS(10*1000)));

    create_task("Maze", 2, maze_task,
            configMINIMAL_STACK_SIZE * 6, nullptr);

    // create_task("Setter", 2, set_target_position_task,
    //         configMINIMAL_STACK_SIZE * 3, nullptr);
    // create_task("Distance", 3, distance_sensors_task,
    //         configMINIMAL_STACK_SIZE * 2, nullptr);

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
