#pragma once

#include <cstdint>
#include <cstddef>

namespace logging {

/*
 * General container for a buffer with data.
 * It points to data that is allocated either statically or dynamically.
 *
 * If is_owner=true, than it means that the receiver of the buffer
 * should take care of deleting it. If is_owner=false, than it means
 * that the data will be vaild as long as needed ==> UNSAFE*!
 * (* unsafe if log() is asynchronious)
 *
 * So more often than not we should use dynamically alocated buffers.
 */
struct Buffer {
    uint8_t *data;
    size_t size;
    bool is_owner;

    // convenient constructor from static arrays
    // deducts the size of the array using templates
    // (with any compiler optimizations this gets optimized away)
    template<size_t size>
    static Buffer from_static(uint8_t (&buf)[size]) {
        return Buffer{buf, size, false};
    }
};

// this will lazily initialise logging when first called
// will return false ignoring the message if the queue is full
// (note: on false the Buffer(is_owner=true) will not be deleted!)
bool log(Buffer buf);

// to be called from ISR routines; when should_yield=true,
// a context switch should be performed at the end of ISR
//
// this function will only work if logging has been already initialised,
// this it to prevent problems when ISR fires before RTOS is started
// when not initialised it returns false
//
// should_yield is only set to true if context switch required,
// it is not set to false, so that one variable can be used for logica OR
bool log_from_isr(Buffer buf, bool &should_yield);

} // namespace logging

