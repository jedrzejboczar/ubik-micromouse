/**
 * @file maze_wanderer.h
 * @brief The actual movement control in the maze
 * @author Jędrzej Boczar
 * @version 0.0.1
 * @date 2019-02-21
 */
#pragma once

#include "ubik/common/robot_parameters.h"

namespace maze::wanderer {


/// Distance required to move to the middle of a cell when standing with back to a wall
constexpr float DISTANCE_MOVE_AWAY_FROM_WALL = 0.020;

constexpr float VEL_LIN_LOW = 0.10;
constexpr float VEL_LIN_MAX = 0.25;
constexpr float VEL_ANG_LOW = constants::c_deg2rad(90);
constexpr float VEL_ANG_MAX = constants::c_deg2rad(220);

constexpr float ACC_LIN_LOW = 0.10;
constexpr float ACC_LIN_MAX = 0.20;
constexpr float ACC_ANG_LOW = constants::c_deg2rad(90);
constexpr float ACC_ANG_MAX = constants::c_deg2rad(180);


/** @brief Used to disable any movement when we e.g. want to move the robot by hand */
void set_enabled(bool enabled);

/** @brief Prepares for the exploration
 *
 * It is assumed that we have a wall behind us and that there are walls at left and right.
 * This function prepares necessary movements to have a clean start of exploration.
 * */
void prepare_starting_from_wall_behind();

} // namespace maze::wanderer
