#define MAZE_TESTING

#include "maze.h"
#include "maze.cpp"


#include "maze_generator.h"

size_t size = 0;
maze::Cell *real_cells;

Directions maze::read_walls(maze::Position pos) {
    if (real_cells == nullptr)
        return Directions();
    maze::Cell *cell = &real_cells[pos.y * size + pos.x];
    return cell->walls;
}
Dir maze::choose_best_direction(Directions possible) {
    if (possible & Dir::N) return Dir::N;
    if (possible & Dir::S) return Dir::S;
    if (possible & Dir::E) return Dir::E;
    if (possible & Dir::W) return Dir::W;
    assert(0);
}

void maze::move_in_direction(Dir dir) {
    std::printf("Moving in direction %s\n",
            dir == Dir::N ? "N" :
            dir == Dir::S ? "S" :
            dir == Dir::E ? "E" :
            dir == Dir::W ? "W" : "ERROR");
}

int main()
{
    size = 4;
    const float mid = (size - 1) / 2.0;
    auto from = maze::Position(0, 0);
    auto to = maze::TargetPosition(int(mid)+1, int(mid)+1);
    // auto to = maze::TargetPosition(size-1, size-1);

    maze::Cell cells[size * size];
    StaticStack<maze::Position, 64> stack;
    maze::Maze maze(size, size, cells, stack, from);

    // RANDOM MAZE GENERATION
    srand(time(NULL));
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
    maze::Maze real_maze(size, size, real_cells, dummy_stack, {0, 0});
    real_maze.print(from, maze::Position(to.x, to.y));
    std::cout << "Press enter to start." << std::endl;
    std::cin.get();
    // RANDOM MAZE GENERATION

    bool success = maze.go_to(to);
    if (!success)
        std::cerr << "ERROR: could not finish maze!";

    maze.print(maze.position(), maze::Position(to.x, to.y));
}
