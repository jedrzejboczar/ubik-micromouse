#pragma once

#include <cstdint>
#include <cassert>

/*
 * Enum representing geographical direction, that also allows for quite
 * easy indexing, e.g.:
 *   for (Dir dir = Dir::FIRST; dir < Dir::COUNT; ++dir) { ... }
 */
enum class Dir: uint8_t {
    N = 0, S, E, W,
    COUNT,
    FIRST = 0
};

Dir& operator++(Dir& dir) {
    return dir = static_cast<Dir>(static_cast<uint8_t>(dir) + 1);
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
