/**
 * @file maze_wanderer.cpp
 * @brief The actual movement control in the maze
 * @author Jędrzej Boczar
 * @version 0.0.1
 * @date 2019-02-22
 */
#include "FreeRTOS.h"
#include "task.h"

#include "maze_wanderer.h"

#include "maze.h"
#include "maze_definitions.h"
#include "ubik/common/robot_parameters.h"
#include "ubik/common/distance_sensors.h"
#include "ubik/movement/controller.h"

#include "ubik/logging/logging.h"


namespace maze {
namespace wanderer {

using namespace movement;


/// We need to keep track of the direction as maze solver doesn't care
static Dir current_dir = Dir::N;
static bool enabled = false;

static void not_enabled_warning() {
    logging::printf(80, "[WARNING] [maze] Maze walker not enabled, cannot prepare!\n");
}


void set_enabled(bool _enabled) {
    enabled = _enabled;
}

void prepare_starting_from_wall_behind() {
    if (!enabled) {
        not_enabled_warning();
        return;
    }

    // calibrate sensors to the walls at sides while we are at the middle
    logging::printf(80, "[maze] Performing side walls calibration\n");
    correction::side_walls.calibrate();
    // correction::side_walls.set_enabled(true);

    // calibrate front sensors by carrefully turning to the wall
    logging::printf(80, "[maze] Performing front walls calibration sequence...\n");
    controller::wait_until_finished(
            controller::move(Rotate(constants::RIGHT_ANGLE, VEL_ANG_LOW, ACC_ANG_LOW, 0)));;
    correction::front_walls.calibrate();
    controller::wait_until_finished(
            controller::move(Rotate(-constants::RIGHT_ANGLE, VEL_ANG_LOW, ACC_ANG_LOW, 0)));

    // move away from the wall that we have behind to reach center of the cell
    logging::printf(80, "[maze] Moving away from wall\n");
    controller::wait_until_finished(
            controller::move(Line(DISTANCE_MOVE_AWAY_FROM_WALL, VEL_LIN_LOW, ACC_LIN_LOW, 0)));
}

} // namespace wanderer

/*** Implementation of maze interface functions *******************************/
using namespace wanderer;

/*
 * Example distance sensors readings in different parts of maze:
 * - [3732  230   46  641  353 3927] - start position (back to wall)
 * - [3776  297   49  647  456 3926] - middle of cell (walls left, right), cell forward has walls: left, front
 * - [   2 2615 2023 1916 3825 3925] - middle of cell (walls left, front), wall 2 cells on the right (so invisible)
 * - [   0  782  335  944 1204 3932] - between two cells, cell last has walls: left, right; cell next has walls: left, front
 */
Directions read_walls(Position pos) {
    const int threshold = 1000;
    Directions walls;

    // sides & front
    uint8_t sensors =
        spi::gpio::DISTANCE_SENSORS[0] |
        spi::gpio::DISTANCE_SENSORS[1] |
        spi::gpio::DISTANCE_SENSORS[4] |
        spi::gpio::DISTANCE_SENSORS[5];
    auto readings = distance_sensors::read(sensors);

    if (readings.sensor[0] > threshold)
        walls |= increment(current_dir, -1); // right
    if (readings.sensor[5] > threshold)
        walls |= increment(current_dir, 1); // left
    // front
    if (readings.sensor[1] > threshold && readings.sensor[4] > threshold)
        walls |= current_dir;

    logging::printf(60, "[maze] Current position (%d, %d)\n", pos.x, pos.y);
    vTaskDelay(10);

    logging::printf(100, "[maze] Updated walls: [%s %s %s %s] <- [%4d %4d %4d %4d %4d %4d]\n",
            walls & Dir::N ? to_string(Dir::N) : "_",
            walls & Dir::E ? to_string(Dir::E) : "_",
            walls & Dir::S ? to_string(Dir::S) : "_",
            walls & Dir::W ? to_string(Dir::W) : "_",
            readings.sensor[0],
            readings.sensor[1],
            readings.sensor[2],
            readings.sensor[3],
            readings.sensor[4],
            readings.sensor[5]
            );

    return walls;
}

Dir choose_best_direction(Directions possible) {
    // as we now move step by step, choose any direction
    for (Dir dir = Dir::FIRST; dir < Dir::COUNT; ++dir)
        if (possible & dir)
            return dir;
    logging::printf(50, "[maze] No direction available!\n");
    return Dir::NONE;
}


void move_in_direction(Dir dir) {
    if (!enabled) {
        not_enabled_warning();
        return;
    }

    // check how much we need to turn
    int turn = difference(current_dir, dir);

    logging::printf(80, "[maze] Moving in direction %s (turn = %d)\n", to_string(dir), turn);

    if (turn != 0) {
        controller::wait_until_finished(controller::move(
                    Rotate(turn * constants::RIGHT_ANGLE, VEL_ANG_MAX, ACC_ANG_MAX, 0)));
        current_dir = dir;
    }

    // move to the next cell
    movement::correction::side_walls.set_enabled(true);
    controller::wait_until_finished(controller::move(
                Line(CELL_EDGE_LENGTH, VEL_LIN_MAX, ACC_LIN_MAX, 0)));
    movement::correction::side_walls.set_enabled(false);
}

} // namespace maze
