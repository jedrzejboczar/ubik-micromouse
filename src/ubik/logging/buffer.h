#pragma once

#include <cstdint>
#include <cstddef>

/*
 * General container for a buffer with data (as bytes).
 * It points to data that is allocated either statically or dynamically.
 *
 * It doesn not provide any higher abstraction, has all data members public
 * and does not manage any resources by itself. Just a plain old structure,
 * so be careful.
 *
 * If is_owner=true, than it means that the receiver of the buffer
 * should take care of deleting it.
 * If is_owner=false, than it means that the receiver will only reference
 * the data and the user ensures that the data will be vaild for as long
 * as needed ==> UNSAFE!
 *
 * So more often than not we should use dynamically alocated buffers.
 */
struct Buffer {
    uint8_t *data;
    size_t size;
    bool is_owner;

    Buffer(): data(nullptr), size(0), is_owner(false) {}
    Buffer(uint8_t *data, size_t size, bool is_owner):
        data(data), size(size), is_owner(is_owner) {}

    // convenient constructor from static arrays
    // deducts the size of the array using templates
    // (with any compiler optimizations this gets optimized away)
    template<size_t size>
    static Buffer from_static(uint8_t (&buf)[size]) {
        return Buffer{buf, size, false};
    }

    // most often we want dynamic buffer that passes ownership
    // but sometimes (theoretically) we could like is_owner=false
    static Buffer dynamic(size_t size, bool is_owner=true) {
        uint8_t *buf = new uint8_t[size];
        return Buffer{buf, size, is_owner};
    }

    void delete_if_owned() {
        if (is_owner)
            delete [] data;
        // for security:
        // invalidate Buffer structure by reseting everything
        data = nullptr;
        size = 0;
        is_owner = false;
    }
};
