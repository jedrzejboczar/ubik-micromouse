#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cassert>
#include <limits>
#include <algorithm>

#include "stack.h"
#include "directions.h"

#define MAZE_TESTING 1

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

// TODO: must be provided
#if MAZE_TESTING == 1
// Directions read_walls(Position pos); // for testing defined later
Dir choose_best_direction(Directions possible);
void move_in_direction(Dir dir);
#endif

namespace maze {

/*
 * Maze consists of square cells, each can have walls on each side (4 directions).
 * For each cell we define its weight representing the manhatan distance to target.
 */
typedef uint8_t weight_t;

struct Cell {
    weight_t weight;
    Directions walls;
};

/*
 * For indexing cells in maze we use two cartesian coordinates (signed
 * for easier subtraction).
 */
struct Position {
    int8_t x;
    int8_t y;
    Position(): Position(-1, -1) {}
    Position(int8_t x, int8_t y): x(x), y(y) {}
};

Directions read_walls(Position pos);

/*
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

/*
 * Manhatan distance / taxicab metric / L2 metric
 * This is a distance when we can only move either vertically or horizontally
 * (like on Manhatan streets). This also applies to the maze.
 */
size_t manhatan_distance(Position pos1, Position pos2) {
    return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y);
}

float manhatan_distance(TargetPosition pos1, TargetPosition pos2) {
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
    Maze(size_t X, size_t Y, Cell cells[], Stack<Position> &stack);

    // move between two positions and return end position
    // (Maze doesn't hold current position)
    Position go_from_to(Position from, TargetPosition to);

#if MAZE_TESTING == 1
    void print(Position current, Position target);
#endif

private:
    // structures allocated somewhere else, not owned
    size_t X, Y;
    Cell *cells;
    Stack<Position> &stack;

    // cell indexing
    Cell &cell(size_t x, size_t y);
    const Cell &cell(size_t x, size_t y) const;
    Cell &cell(Position pos);
    const Cell &cell(Position pos) const;
    size_t n_cells() const;

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


// EXAMPLE
#if MAZE_TESTING == 1

#include <cstdio>
// for print()
#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>

#include "maze_generator.h"

size_t size = 0;
maze::Cell *real_cells;

Directions maze::read_walls(maze::Position pos) {
    if (real_cells == nullptr)
        return Directions();
    maze::Cell *cell = &real_cells[pos.y * size + pos.x];
    return cell->walls;
}
Dir choose_best_direction(Directions possible) {
    if (possible & Dir::N)
        return Dir::N;
    if (possible & Dir::S)
        return Dir::S;
    if (possible & Dir::E)
        return Dir::E;
    if (possible & Dir::W)
        return Dir::W;
    assert(0);
}

void move_in_direction(Dir dir) {
    std::printf("Moving in direction %s\n",
            dir == Dir::N ? "N" :
            dir == Dir::S ? "S" :
            dir == Dir::E ? "E" :
            dir == Dir::W ? "W" : "ERROR");
}

int main()
{
    size = 16;
    const float mid = (size - 1) / 2.0;
    auto from = maze::Position(0, 0);
    auto to = maze::TargetPosition(mid, mid);

    maze::Cell cells[size * size];
    StaticStack<maze::Position, 64> stack;
    maze::Maze maze(size, size, cells, stack);

    // RANDOM MAZE GENERATION
    maze_gen::MazeGenerator generator;
    generator.create(size);
    real_cells = new maze::Cell[size * size];
    for (int y = 0; y < generator._s; y++) {
        int yy = y * generator._s;
        for (int x = 0; x < generator._s; x++) {
            uint8_t b = generator._world[x + yy];
            // change the Y addressing as they used standard gui addresing (y=0 at top)
            if( !( b & maze_gen::NOR ) ) real_cells[(size - y - 1) * size + x].walls |= Dir::N;
            if( !( b & maze_gen::SOU ) ) real_cells[(size - y - 1) * size + x].walls |= Dir::S;
            if( !( b & maze_gen::EAS ) ) real_cells[(size - y - 1) * size + x].walls |= Dir::E;
            if( !( b & maze_gen::WES ) ) real_cells[(size - y - 1) * size + x].walls |= Dir::W;
        }
    }
    StaticStack<maze::Position, 1> dummy_stack;
    maze::Maze real_maze(size, size, real_cells, dummy_stack);
    real_maze.print(from, maze::Position(to.x, to.y));
    std::cout << "Press enter to start." << std::endl;
    std::cin.get();
    // RANDOM MAZE GENERATION

    maze::Position initial = maze::Position(0, 0);
    // maze::TargetPosition target = maze::TargetPosition(mid, mid);
    maze::TargetPosition target = maze::TargetPosition(size, size);
    maze::Position current = maze.go_from_to(initial, target);

    maze.print(current, maze::Position(target.x, target.y));
}

#endif
