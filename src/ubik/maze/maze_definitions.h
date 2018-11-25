#pragma once

#include "maze.h"

namespace maze {

// Robotic Arena 2018:
// http://www.roboticarena.pl/media/filer_public/c5/4e/c54e7f8c-6a2f-4a1d-9a84-5bb422acb9ed/micromouse16x16.pdf
static constexpr float CELL_WIDTH = 0.012;
static constexpr float CELL_EDGE_LENGTH = 0.180;
static constexpr float CELL_INNER_EDGE_LENGTH = CELL_EDGE_LENGTH - CELL_WIDTH;
static constexpr float CELL_OUTER_EDGE_LENGTH = CELL_EDGE_LENGTH + CELL_WIDTH;

const Position START_POSITION = Position{0, 0};

} // namespace maze
