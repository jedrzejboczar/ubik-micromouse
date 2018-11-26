#pragma once

#include <cstdint>
#include <limits>


/*
 * Enum representing geographical direction, that also allows for quite
 * easy indexing, e.g.:
 *   for (Dir dir = Dir::FIRST; dir < Dir::COUNT; ++dir) { ... }
 */
enum class Dir: uint8_t {
    N = 0, E, S, W,
    COUNT,
    FIRST = 0,
    NONE = std::numeric_limits<uint8_t>::max() // used to signalise errors
};

static inline Dir& operator++(Dir& dir) {
    return dir = static_cast<Dir>(static_cast<uint8_t>(dir) + 1);
}

/*
 * Calculates the number of right-angle turns needed to go from 'from' to `to`.
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

/*
 * Increment direction by the given number of turns by right angle. Left is positive.
 */
static inline Dir increment(Dir dir, int increment) {
    int new_dir = static_cast<int>(dir) - increment;
    // use Python-like modulo (as C modulo is different than in Python)
    // this should yield currect results (always positive)
    new_dir = ((new_dir % 4) + 4) % 4;
    return static_cast<Dir>(new_dir);
}

static inline Dir opposite(Dir dir) {
    switch (dir) {
        case Dir::N: return Dir::S;
        case Dir::E: return Dir::W;
        case Dir::S: return Dir::N;
        case Dir::W: return Dir::E;
        default: return Dir::NONE;
    }
}


/*
 * Represents a set of directions: zero, one or more.
 * Manages the underlying bitset representation.
 * Overloads some binary operators for OR-ing, AND-ing and negating directions.
 */
class Directions {
public:
    Directions(): dirs(0) {  }
    Directions(Dir dir):
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
