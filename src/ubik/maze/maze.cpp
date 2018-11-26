#include "maze.h"

#if defined(MAZE_TESTING)
#include <iostream>
#include <thread>
#include <iomanip>
#include <string>
#include <chrono>
#elif PRINT_MAZE == 1
#include "ubik/logging/logging.h"
#endif

namespace maze {

Maze::Maze(size_t X, size_t Y, Cell cells[], Stack<Position> &stack, Position start_pos):
    X(X), Y(Y), cells(cells), stack(stack), current_pos(start_pos)
{
    add_borders_walls();
}

bool Maze::go_to(TargetPosition to) {
    init_weights_to_target(to);

    while (cell(current_pos).weight > 0) {
        // analyze current sensors readings (or from near past)
        update_walls(current_pos, read_walls(current_pos));
        flood_fill(current_pos);
        // get all the directions that have minimal weight
        Directions available_directions = reachable_neighbours(current_pos);
        Directions considered_directions = lowest_weight_directions(current_pos, available_directions);

        // choose a direction based on robot orientation and other possible conditions
        Dir dir = choose_best_direction(considered_directions);
        if (dir == Dir::NONE) // if there is no avialable direction, then return
            return false;

        // perform the movement
        move_in_direction(dir);

        // [>*** OR ***<]
        // // send them to the controller and allow it to choose priority
        // send_available_directions(considered_directions);
        // // wait until we have new reliable data about the walls (basically, when we are at next frame)
        // wait_for_new_cell_walls();

#if defined(MAZE_TESTING)
        std::cout << std::string(X * 6, '-') << std::endl;
        print(current_pos, Position(to.x, to.y));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
#elif PRINT_MAZE == 1
        print(current_pos, Position(to.x, to.y));
#endif

        // update current position
        current_pos = neighbour(current_pos, dir);
    }

    return true;
}

Cell& Maze::cell(size_t x, size_t y) {
    return cells[y * Y + x];
}

const Cell& Maze::cell(size_t x, size_t y) const {
    return cells[y * Y + x];
}

Cell& Maze::cell(Position pos) {
    return cell(pos.x, pos.y);
}

const Cell& Maze::cell(Position pos) const {
    return cell(pos.x, pos.y);
}

size_t Maze::n_cells() const {
    return X * Y;
}

void Maze::add_borders_walls() {
    for (size_t x = 0; x < X; x++) {
        for (size_t y = 0; y < Y; y++) {
            Directions &walls = cell(x, y).walls;
            if (x ==   0) walls |= Dir::W;
            if (y ==   0) walls |= Dir::S;
            if (x == X-1) walls |= Dir::E;
            if (y == Y-1) walls |= Dir::N;
        }
    }
}

void Maze::init_weights_to_target(TargetPosition target) {
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

void Maze::update_walls(Position pos, Directions walls) {
    cell(pos).walls |= walls;
}

bool Maze::is_in_maze(Position pos) const {
    return (0 <= pos.x) && (pos.x < static_cast<int8_t>(X))
        && (0 <= pos.y) && (pos.y < static_cast<int8_t>(Y));
}

Position Maze::neighbour(Position pos, Dir dir) const {
    switch (dir) {
        case Dir::N: pos.y += 1; break;
        case Dir::S: pos.y -= 1; break;
        case Dir::E: pos.x += 1; break;
        case Dir::W: pos.x -= 1; break;
        default: configASSERT(0); break;
    }
    return pos;
}

Directions Maze::reachable_neighbours(Position pos) const {
    Directions available = ~cell(pos).walls;
    // remove invalid
    for (Dir dir = Dir::FIRST; dir < Dir::COUNT; ++dir)
        if (!is_in_maze(neighbour(pos, dir)))
            available &= ~Directions(dir);
    return available;
}

weight_t Maze::lowest_weight(Position pos, Directions neighbours) const {
    weight_t min_weight = std::numeric_limits<weight_t>::max();
    for (Dir dir = Dir::FIRST; dir < Dir::COUNT; ++dir)
        if (neighbours & dir) { // if we consider neighbour in this direction
            weight_t weight = cell(neighbour(pos, dir)).weight;
            min_weight = std::min(min_weight, weight);
        }
    return min_weight;
}

Directions Maze::lowest_weight_directions(Position pos, Directions possible) {
    weight_t min_weight = lowest_weight(pos, possible);
    for (Dir dir = Dir::FIRST; dir < Dir::COUNT; ++dir)
        if (possible & dir)
            // remove from possible if weight is not lowest
            if (cell(neighbour(pos, dir)).weight != min_weight)
                possible &= ~Directions(dir);
    return possible;
}

void Maze::push_neigbours(Stack<Position> &stack, Position pos, Directions neighbours) {
    for (Dir dir = Dir::FIRST; dir < Dir::COUNT; ++dir)
        if (neighbours & dir)
            stack.push(neighbour(pos, dir));
}

void Maze::flood_fill(Position pos) {
    // start by pushing current position onto the stack
    stack.empty();
    stack.push(pos);
    // continue until the stack is empty
    while (!stack.is_empty()) {
        pos = stack.pop();
        // omit this cell if it is one of targets (TODO: is this for sure ok?)
        bool is_target = cell(pos).weight == 0;
        if (!is_target) {
            // find the lowest weight across reachable neighbours
            Directions neighbours = reachable_neighbours(pos);
            weight_t min_weight = lowest_weight(pos, neighbours);
            // push the neighbours if the weights around this cell are not consistent
            bool weights_consistent = cell(pos).weight == min_weight + 1;
            if (!weights_consistent) {
                cell(pos).weight += 1;
                push_neigbours(stack, pos, neighbours);
#if defined(MAZE_TESTING)
                std::cout << std::string(X * 6, '-') << std::endl;
                print(Position(-1, -1), pos);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
#endif
            }
        }
    }
}


#if defined(MAZE_TESTING)
void Maze::print(Position current, Position target) {
    std::ostringstream ss;

    for (int y = Y-1; y >= 0; --y) {
        for (int x = 0; x < X; x++)
            ss << "+" << ((cell(x, y).walls & Dir::N) ? "----" : "    ") << "+";
        ss << "\n";
        for (int x = 0; x < X; x++) {
            bool target_now = target.x == x && target.y == y;
            bool current_now = current.x == x && current.y == y;
            ss << ((cell(x, y).walls & Dir::W) ? "|" : " ")
                << std::setw(3) << (int) cell(x, y).weight
                << (current_now ? "@" : target_now ? "$" : " ")
                << ((cell(x, y).walls & Dir::E) ? "|" : " ");
        }
        ss << "\n";
        for (int x = 0; x < X; x++)
            ss << "+" << ((cell(x, y).walls & Dir::S) ? "----" : "    ") << "+";
        ss << "\n";
    }

    std::cout << ss.str() << std::endl;
}
#elif PRINT_MAZE == 1
void Maze::print(Position current, Position target) {
    // calculate the buffer size required
    int required_size =
        // horizontal size + \n
        ( X * 6 + 1 ) *
        // vertical size = number of lines
        ( Y * 3 )
        + 1 + 10; // null byte + safety margin?

    // refuse to print maze if we have to little heap available
    constexpr int min_free_heap= 300;
    int available_heap = xPortGetFreeHeapSize();
    if (required_size > available_heap - min_free_heap) {
        logging::printf(70, "Not enough heap for printing maze! required=%d\n",
                required_size);
        return;
    }

    // to avoid problems with memory usage, block all logging and print in blocking manner
    logging::lock();
    auto msg = logging::Msg::dynamic(required_size);

    // write to the buffer
    char *buf = msg.as_chars();
    for (int y = Y-1; y >= 0; --y) {
        for (int x = 0; x < int(X); x++) {
            *buf++ = '+';
            char sep = (cell(x, y).walls & Dir::N) ? '-' : ' ';
            for (int i = 0; i < 4; i++)
                *buf++ = sep;
            *buf++ = '+';
        }
        *buf++ = '\n';
        for (int x = 0; x < int(X); x++) {
            bool target_now = target.x == x && target.y == y;
            bool current_now = current.x == x && current.y == y;
            *buf++ = (cell(x, y).walls & Dir::W) ? '|' : ' ';
            buf += sprintf(buf, "%3d", cell(x, y).weight);
            *buf++ = current_now ? '@' : target_now ? '$' : ' ';
            *buf++ = (cell(x, y).walls & Dir::E) ? '|' : ' ';
        }
        *buf++ = '\n';
        for (int x = 0; x < int(X); x++) {
            *buf++ = '+';
            char sep = (cell(x, y).walls & Dir::S) ? '-' : ' ';
            for (int i = 0; i < 4; i++)
                *buf++ = sep;
            *buf++ = '+';
        }
        *buf++ = '\n';
    }

    // print the message and wait some time, last thing we want is
    // allocating more than one maze printing buffer
    logging::log_blocking(msg);
    logging::unlock();
}
#endif

} // namespace maze
