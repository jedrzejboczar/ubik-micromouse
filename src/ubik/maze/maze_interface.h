/**
 * @file maze_interface.h
 * @brief Interface between maze solver and the real movement
 * @author Jędrzej Boczar
 * @version 0.0.1
 * @date 2019-02-21
 *
 * Defines the interface functions that must be implemented in oreder
 * for the maze solver to be able to actually move the robot.
 */
#pragma once

#include "maze_cells.h"

namespace maze {

/**
 * @brief Used for finding walls
 * @param pos current position of maze solver
 * @return directions at which walls have been found
 *
 * The walls are used for updating (not overriding) current knowledge.
 */
Directions read_walls(Position pos);

/**
 * @brief Pick best direction (depending on movement direction, etc.)
 * @param possible the directions from which to pick one
 */
Dir choose_best_direction(Directions possible);

/** @brief Perform movement in the given direction */
void move_in_direction(Dir dir);

} // namespace maze
