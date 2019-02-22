/**
 * @file maze_cells.h
 * @brief Representation of cells and positions
 * @author Jędrzej Boczar
 * @version 0.0.1
 * @date 2019-02-21
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "directions.h"

namespace maze {

/** @brief The manhatan distance to target */
typedef uint8_t weight_t;

/** @brief Information that changes while exploring
 *
 * Maze consists of square cells, each can have walls on each side (4 directions);
 * for each cell we define its weight representing the manhatan distance to target.
 */
struct Cell {
    weight_t weight;
    Directions walls;
};

 /** @brief Used for indexing maze cells
  *
  * For indexing cells in maze we use two cartesian coordinates;
  * signed for easier subtraction.
  * */
struct Position {
    int8_t x;
    int8_t y;
    Position(): Position(-1, -1) {}
    Position(int8_t x, int8_t y): x(x), y(y) {}
};

/** @brief Used for indexing "between" cells
 *
 * To index an edge a fractional position can be used, e.g. (2.5, 2.5).
 * This is to be used for manhatan distance calculations when we want to
 * any of 2x2 cells (than all of them should have 0 weight).
 */
struct TargetPosition {
    float x;
    float y;
    TargetPosition(float x, float y): x(x), y(y) {}
    // allow because we don't loose precision
    TargetPosition(Position pos): x(pos.x), y(pos.y) {}
};

} // namespace maze
