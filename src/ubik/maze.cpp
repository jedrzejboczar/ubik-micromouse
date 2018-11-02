#include <cstddef>
#include <cstdint>
#include <cstring>
// #include <cstdlib> // TODO: for abs(), deos including this add great overhead?
#include <cmath>
#include <cassert>
#include <limits>
#include <algorithm>

namespace maze {


/******************************************************************************/

/*
 * Some helpful structs.
 * Directions represents a set of directions (can be more than one or all).
 * Walls are used to represent known walls in a cell.
 * Cell has weight corresponding to manhatan distance to target.
 * Position is used to index the 2D representation of maze.
 * TargetPosition is only for choosing the target to allow for indexing edges.
 * manhatan_distance() calculates the L2 metric for 2D vector.
 */
enum class Dir: uint8_t {
    N = 0, S, E, W,
    COUNT,
    FIRST = 0
};

Dir& operator++(Dir& stackID) {
    return stackID = static_cast<Dir>(static_cast<uint8_t>(stackID) + 1);
}

class Directions {
public:
    Directions(): dirs(0) {  }
    Directions(Dir dir):
        dirs(1 << static_cast<uint8_t>(dir)) { assert(dir < Dir::COUNT); }

    operator bool() const {
        return dirs != 0;
    }
    Directions operator ~() const {
        return (~dirs) & mask;
    }
    Directions& operator |=(Directions other) {
        dirs |= other.dirs;
        return *this;
    }
    Directions& operator &=(Directions other) {
        dirs &= other.dirs;
        return *this;
    }
private:
    uint8_t dirs;

    static constexpr uint8_t mask = (1 << static_cast<uint8_t>(Dir::COUNT)) - 1;
    static_assert(mask == 0b1111, "Directions bitmask should most probably be 0b1111");

    // allow freely for conversion only in private members
    Directions(uint8_t dirs): dirs(dirs) {  }
};

inline Directions operator|(Directions lhs, const Directions& rhs) {
  lhs |= rhs;
  return lhs;
};
inline Directions operator&(Directions lhs, const Directions& rhs) {
  lhs &= rhs;
  return lhs;
};

typedef uint8_t weight_t;

struct Cell {
    weight_t weight;
    Directions walls;
};

struct Position {
    int8_t x;
    int8_t y;
    Position(int8_t x, int8_t y): x(x), y(y) {}
};
struct TargetPosition {
    float x;
    float y;
    TargetPosition(float x, float y): x(x), y(y) {}
    // allow because we don't loose precision
    TargetPosition(Position pos): x(pos.x), y(pos.y) {}
};

size_t manhatan_distance(Position pos1, Position pos2) {
    return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y);
}
float manhatan_distance(TargetPosition pos1, TargetPosition pos2) {
    return fabsf(pos1.x - pos2.x) + fabsf(pos1.y - pos2.y);
}

/*
 * Stack optimized for small types (takes and returns by copy).
 * For larger types, pointers should be used (e.g. Stack<BigObject*, 100>).
 * On reaching the limit size it will panic!
 * pop() from empty stack will panic!
 */
// interface for usage without knowledge of size
template<typename T>
class Stack {
public:
    virtual void push(T elem) = 0;
    virtual T pop() = 0;
    virtual T top() = 0;
    virtual size_t size() = 0;
    virtual bool is_empty() = 0;
    virtual void empty() = 0;
};
// concrete implementation
template<typename T, size_t size_limit>
class StaticStack: Stack<T> {
    int itop;
    T elements[size_limit];
public:
    StaticStack() {
        empty();
    }
    virtual void push(T elem) override {
        itop++;
        assert(itop < size_limit);
        elements[itop] = elem;
    }
    virtual T pop() override {
        assert(itop >= 0);
        return elements[itop--];
    }
    virtual T top() override {
        assert(itop >= 0);
        return elements[itop];
    }
    virtual size_t size() override {
        return itop + 1;
    }
    virtual bool is_empty() override {
        return size() <= 0;
    }
    virtual void empty() override {
        itop = -1;
    }
};

/******************************************************************************/

/*
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
 */
class Maze {
    // buffers allocated somewhere else
    size_t X, Y;
    Cell *cells;
    Stack<Position> &stack;
public:

    // Construct Maze of given dimensions using given 'cells' buffer.
    // Both 'cells' and 'stack' are referenced only (no construction/destruction)
    // and must be instantiated somewhere else and be valid as long as Maze.
    Maze(size_t X, size_t Y, Cell cells[], Stack<Position> &stack):
        X(X), Y(Y), cells(cells), stack(stack) { }

    Cell &cell(size_t x, size_t y) {
        return cells[y * Y + x];
    }

    Cell &cell(Position pos) {
        return cell(pos.x, pos.y);
    }

    size_t n_cells() const {
        return X * Y;
    }

    void init_weights_to_target(TargetPosition target) {
        // set weights using float and rounding to int
        for (size_t x = 0; x < X; x++) {
            for (size_t y = 0; y < Y; y++)
                cell(x, y).weight = manhatan_distance(Position(x, y), target);
            // make sure that 0 is the lowest weight
            auto cmp = [] (const Cell &c1, const Cell &c2) {
                return c1.weight < c2.weight;
            };
            auto min_weight = std::min_element(cells, cells + n_cells(), cmp)->weight;
            std::for_each(cells, cells + n_cells(),
                    [min_weight] (Cell &c) { c.weight -= min_weight; });
        }
    }

    // or should use logical OR?
    void update_walls(Position pos, Directions walls) {
        cell(pos).walls = walls;
    }

    bool is_in_maze(Position pos) {
        return (0 <= pos.x) && (pos.x < static_cast<int8_t>(X))
            && (0 <= pos.y) && (pos.y < static_cast<int8_t>(Y));
    }

    Position neighbour(Position pos, Dir dir) {
        switch (dir) {
            case Dir::N: pos.y += 1; break;
            case Dir::S: pos.y -= 1; break;
            case Dir::E: pos.x += 1; break;
            case Dir::W: pos.x -= 1; break;
            default: assert(0); break;
        }
        return pos;
    }

    Directions available_neighbours(Position pos) {
        Directions avalable = ~cell(pos).walls;
        // remove invalid
        for (Dir dir = Dir::FIRST; dir < Dir::COUNT; ++dir) {
            if (!is_in_maze(neighbour(pos, dir)))
                avalable &= ~Directions(dir);
        }
        return avalable;
    }

    weight_t lowest_weight(Position pos, Directions neighbours)  {
        weight_t min_weight = std::numeric_limits<weight_t>::max();
        for (Dir dir = Dir::FIRST; dir < Dir::COUNT; ++dir) {
            if (neighbours & dir) { // if we consider neighbour in this direction
                weight_t weight = cell(neighbour(pos, dir)).weight;
                min_weight = std::min(min_weight, weight);
            }
        }
    }

    void flood_fill(Position pos) {
        stack.empty();
        stack.push(pos);
        while (!stack.is_empty()) {
            pos = stack.pop();
            Directions neighbours = available_neighbours(pos);
            // auto min_weight = std::min_element(cells, cells + n_cells(), cmp)->weight;
        }
    }
};

Directions read_walls();

Position go_from_to(Maze &maze, Position from, TargetPosition to) {
    maze.init_weights_to_target(to);
    Position pos = from;

    while (maze.cell(pos).weight > 0) {
        // analyze current sensors readings (or from near past)
        maze.update_walls(pos, read_walls());
        maze.flood_fill(pos);
        // get all the directions that have minimal weight
        Directions available_directions = maze.best_directions(pos);

        // choose a direction based on robot orientation and other possible conditions
        Directions dir = choose_best_direction(available_directions);
        move_in_direction(dir);
        /**** OR ****/
        // send them to the controller and allow it to choose priority
        send_available_directions(available_directions);
        // wait until we have new reliable data about the walls (basically, when we are at next frame)
        wait_for_new_cell_walls();

        // update current position
        pos = get_position_in_direction(pos, dir);
    }

    return pos;
}




} // namespace maze
