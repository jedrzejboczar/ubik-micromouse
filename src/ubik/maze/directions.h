/**
 * @file directions.h
 * @brief Simple classes for dealing with geographical directions
 * @author Jędrzej Boczar
 * @version 0.0.1
 * @date 2019-02-22
 */
#pragma once

#include <cstdint>
#include <limits>


/**
 * @brief Geographical direction representation
 *
 * Enum for working with geographical directions; it allows for quite
 * easy iteration over all directions, e.g.:
 * @code
 * for (Dir dir = Dir::FIRST; dir < Dir::COUNT; ++dir) { ... }
 * @endcode
 */
enum class Dir: uint8_t {
    N = 0, E, S, W,
    COUNT,
    FIRST = 0,
    NONE = std::numeric_limits<uint8_t>::max() ///< used to signalise errors
};

/** @brief Convenient overload for incrementing (to avoid typing the static_casts) */
static inline Dir& operator++(Dir& dir) {
    return dir = static_cast<Dir>(static_cast<uint8_t>(dir) + 1);
}

/** @brief Converts enum values to string */
static inline const char* to_string(Dir dir) {
    const char *strings[] = {"N", "S", "E", "W"};
    if (static_cast<uint8_t>(dir) >= static_cast<uint8_t>(Dir::COUNT))
        return "!";
    else
        return strings[static_cast<uint8_t>(dir)];
}

/**
 * @brief Difference between directions
 *
 * Calculates the number of right-angle turns needed to go from @p from to @p to.
 * Turning left (e.g. from N to W) is positive.
 */
static inline int difference(Dir from, Dir to) {
    int difference =  static_cast<int>(from) - static_cast<int>(to);
    if (difference > 2)
        difference -= 4;
    else if (difference < -1)
        difference += 4;
    return difference;
}

/**
 * @brief Modulo-like increment of direction
 *
 * Increments direction by the given number of right-angle turns.
 * Positive @p increment means turning left.
 */
static inline Dir increment(Dir dir, int increment) {
    int new_dir = static_cast<int>(dir) - increment;
    // use Python-like modulo (as C modulo is different than in Python)
    // this should yield currect results (always positive)
    new_dir = ((new_dir % 4) + 4) % 4;
    return static_cast<Dir>(new_dir);
}

/** @brief Opposite direction (e.g. S is opposite to N) */
static inline Dir opposite(Dir dir) {
    switch (dir) {
        case Dir::N: return Dir::S;
        case Dir::E: return Dir::W;
        case Dir::S: return Dir::N;
        case Dir::W: return Dir::E;
        default: return Dir::NONE;
    }
}


/** @brief Set of directions: zero, one or more. Has bitset-like interface.
 *
 * The class manages the underlying bitset representation.
 * Overloads some binary operators for OR-ing, AND-ing and negating directions.
 */
class Directions {
public:
    Directions(): dirs(0) {  }
    Directions(Dir dir):  // TODO: remove configASSERT? have some interface?
        dirs(1 << static_cast<uint8_t>(dir)) { configASSERT(dir < Dir::COUNT); }

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

static inline Directions operator|(Directions lhs, const Directions& rhs) {
  lhs |= rhs;
  return lhs;
}

static inline Directions operator&(Directions lhs, const Directions& rhs) {
  lhs &= rhs;
  return lhs;
}
