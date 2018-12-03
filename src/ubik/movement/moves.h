#pragma once

#include <tuple>
#include <variant>
#include <type_traits>

#include "trajectory.h"

namespace movement {

/*
 * At first this was used for an interface that should be provided by each Move.
 *
 * class Move {
 * public:
 *     virtual ~Move() {}
 *     virtual void initialise_generator(TrajectoryGenerator &generator) = 0;
 *     virtual Pair convert_position(float generator_position_delta) = 0;
 * };
 *
 * But basically, we needed to pass those moves statically allocated through a
 * FreeRTOS queue (I mean, we _could_ allocate them dynamically, but you know,
 * embedded...).
 *
 * In order to pass polymorphic objects statically, we need some kind of strange
 * object composition with a buffer that can hold the biggest of Move children.
 *
 * Or we could use a union. But to dispatch, we need a struct { MoveType, Move },
 * and basing on MoveType, we would use different field of the union. It is not
 * ideal, ok.
 *
 * Then there is std::variant. It is safer, and more ... elegant.
 * And use std::visit, that is used for dispatching on the variant.
 * Than we can just use duck-typing. If it quacks it is a duck...
 * So just use lambda, e.g. (auto any_move) { any_move.initialise_generator() }
 *
 * So there is no need for inheritance and those pesky virtual tables!
 * The interface is defined by how we use our variant.
 * Maybe it's good, maybe bad, we'll see.
 *
 * IMPORTANT:
 * All the moves must be simple - no strange code in constructors/destructors,
 * no dynamic allocation and so on...
 * This is because of how the FreeRTOS queues work. They copy raw data, so no
 * constructors and destructors are called.
 * I am not sure how _exactly_ this condition should be called. I assume that
 * std::is_trivialy_destructible will be enough.
 * This is because those object may not get destructed? Or maybe they will?
 * My CPP-fu ends here I guess...
 *
 */

typedef std::pair<float, float> Pair;


class Line {
    float distance, vel_desired, acc, vel_final;
public:
    Line(float distance, float vel_desired, float acc, float vel_final);
    void initialise_generator(TrajectoryGenerator &generator);
    Pair convert_position(float generator_position_delta);
};


class Rotate {
    float angle, vel_desired, acc, vel_final;
public:
    Rotate(float angle, float vel_desired, float acc, float vel_final);
    void initialise_generator(TrajectoryGenerator &generator);
    Pair convert_position(float generator_position_delta);
};

/*
 * Moving along an arc puts restrictions on the movement, so `vel_desired`, `acc`
 * and `vel_final` refer to the linear motion parameters.
 *
 * The restriction is:
 *    constant curvature of an arc: k = d_angle / d_s = 1 / R = const
 * So:
 *    k = 1 / R
 *    d_angle = d_s * k
 *    vel_ang = vel_lin * k
 */
class Arc {
    float distance, vel_desired, acc, vel_final;
    float curvature; // k = angle / distance_lin
public:
    Arc(Pair distances, float vel_desired, float acc, float vel_final);
    void initialise_generator(TrajectoryGenerator &generator);
    Pair convert_position(float generator_position_delta);
};


/*
 * TODO: Euler spiral
 * For more gentle turns we should use a combination of euler spirals and arcs.
 * Euler spiral should probably be modeled as Arc with non-constant curvature,
 * probably depending on the ratio vel_current/vel_desired?
 * The important part is to calculate the movement parameters so that we can
 * specify the whole path with just two parameters (as in Arc). This should
 * be calculated in the constructor if possible.
 */


/*
 * Static-sized type that holds either one of Moves or nothing.
 * At this moment, sizes are as follows (use static_assert(sizeof(x) == y) to check):
 *   Line    -> 16
 *   Rotate  -> 16
 *   Arc     -> 20
 *   AnyMove -> 24
 */
using AnyMove = std::variant<Line, Rotate, Arc>;

// maybe it is the right check, maybe not...
static_assert(std::is_trivially_destructible<AnyMove>());


} // namespace movement
