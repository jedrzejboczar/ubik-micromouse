#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <limits>
#include <algorithm>

#define PRINT_MAZE 1

#if defined(MAZE_TESTING)
#include <cassert>
#define configASSERT(x) assert(x)
#else
#include "FreeRTOS.h"
#include "task.h"
#endif

#include "stack.h"
#include "maze_cells.h"
#include "maze_interface.h"

/*******************************************************************************
 *
 *     ^                 N
 *     |       +---+---+---+---+---+
 *     |  4    | 4 | 3 | 2 | 3 | 4 |
 *     |       +---+---+---+---+---+
 *     |  3    | 3 | 2 | 1 | 2 | 3 |
 *     |       +---+---+---+---+---+
 *     |  2  W | 2 | 1 | 0 | 1 | 2 | E
 *   y |       +---+---+---+---+---+
 *     |  1    | 3 | 2 | 1 | 2 | 3 |
 *     |       +---+---+---+---+---+
 *     |  0    | 4 | 3 | 2 | 3 | 4 |
 *     |       +---+---+---+---+---+
 *     |                 S
 *     |         0   1   2   3   4
 *     +--------------------------------->
 *                      x
 *
 * The maze is indexed as shown above (x - column nr, y - row nr).
 * In memory representation is an array of rows.
 * Cell weights represent distance to target (0 is the target).
 * There can be more than one target by choosing half-index (e.g. when
 * searching for a 2x2 square, choose index of the egde in the middle).
 *
 ******************************************************************************/


namespace maze {

/**
 * @brief Manhatan distance / taxicab metric / L2 metric
 *
 * This is a distance when we can only move either vertically or horizontally
 * (like on Manhatan streets); this applies to the maze.
 */
static inline size_t manhatan_distance(Position pos1, Position pos2) {
    return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y);
}

/** @brief manhatan_distance overload for TargetPosition */
static inline float manhatan_distance(TargetPosition pos1, TargetPosition pos2) {
    return fabsf(pos1.x - pos2.x) + fabsf(pos1.y - pos2.y);
}

/*
 * This is a class that implements flood-fill algorithm for traversing a maze.
 * It could be easily extended to support other algorithms, but what for?
 *
 * The Cells and Stack<Position> have to be allocated elsewhere and passed
 * in the constructor, Maze does NOT take ownership of those.
 * This allows for static allocation - embedded FTW!
 * (not as template as we wouldn't like to have 2 Mazes, for mazes 8x8 and 16x16)
 */
class Maze {
public:
    // Construct Maze of given dimensions using given 'cells' buffer.
    // Both 'cells' and 'stack' are referenced only (no construction/destruction)
    // and must be instantiated somewhere else and be valid as long as Maze.
    Maze(size_t X, size_t Y, Cell cells[], Stack<Position> &stack, Position start_pos);

    // move between two positions
    bool go_to(TargetPosition to);

#if defined(MAZE_TESTING) || PRINT_MAZE == 1
    void print(Position current, Position target);
#endif

    Position position() { return current_pos; }
    void set_position(Position pos) { current_pos = pos; }
private:
    // structures allocated somewhere else, not owned
    size_t X, Y;
    Cell *cells;
    Stack<Position> &stack;
    Position current_pos;

    // cell indexing
    Cell &cell(size_t x, size_t y);
    const Cell &cell(size_t x, size_t y) const;
    Cell &cell(Position pos);
    const Cell &cell(Position pos) const;
    size_t n_cells() const;

    void add_borders_walls();
    void init_weights_to_target(TargetPosition target);
    void update_walls(Position pos, Directions walls);
    bool is_in_maze(Position pos) const;
    Position neighbour(Position pos, Dir dir) const;
    Directions reachable_neighbours(Position pos) const;
    weight_t lowest_weight(Position pos, Directions neighbours) const;
    Directions lowest_weight_directions(Position pos, Directions possible);
    void push_neigbours(Stack<Position> &stack, Position pos, Directions neighbours);
    void flood_fill(Position pos);
};


} // namespace maze

