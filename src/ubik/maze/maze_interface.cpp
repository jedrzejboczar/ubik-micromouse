#include "FreeRTOS.h"
#include "task.h"

#include "maze.h"
#include "maze_definitions.h"
#include "ubik/common/robot_parameters.h"
#include "ubik/common/distance_sensors.h"
#include "ubik/movement/controller.h"

#include "ubik/logging/logging.h"


namespace maze {

static constexpr Dir start_dir = Dir::N;
static Dir current_dir = start_dir;

static const char* str(Dir dir) {
    switch (dir) {
        case Dir::N: return "N";
        case Dir::S: return "S";
        case Dir::E: return "E";
        case Dir::W: return "W";
        default: return "!";
    }
}


// this only has to add new walls, so we don't have to move
Directions read_walls(Position pos) {
    const int threshold = 1000;
    Directions walls;

    // add a wall behind us when we start
    if (pos.x == 0 && pos.y == 0)
        walls |= increment(start_dir, 2);

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
            walls & Dir::N ? "N" : "_",
            walls & Dir::E ? "E" : "_",
            walls & Dir::S ? "S" : "_",
            walls & Dir::W ? "W" : "_",
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

// requires N=0, E=1, S=2, W=3

void move_in_direction(Dir dir) {
    const float right_angle = constants::deg2rad(90);
    const float vel_lin = 0.20;
    const float acc_lin = 0.20;
    const float vel_ang = constants::deg2rad(180);
    const float acc_ang = constants::deg2rad(180);

    // check how much we need to turn
    int turn = difference(current_dir, dir);

    logging::printf(80, "[maze] Moving in direction %s (turn = %d)\n", str(dir), turn);

    if (turn != 0) {
        movement::controller::move_rotate(turn * right_angle, vel_ang, acc_ang);
        current_dir = dir;
    }

    // move to the next cell
    movement::controller::move_line(CELL_EDGE_LENGTH, vel_lin, acc_lin);
}


} // namespace maze
