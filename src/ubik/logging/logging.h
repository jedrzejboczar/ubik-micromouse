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
    // most often we want dynamic buffer that passes ownership
    // but sometimes (theoretically) we could like is_owner=false
    static Buffer dynamic(size_t size, bool is_owner=true) {
        uint8_t *buf = new uint8_t[size];
        return Buffer{buf, size, is_owner};
    }
};

struct Msg: Buffer {
    char *as_chars() {
        return reinterpret_cast<char *>(data);
    }
};

/*
 * Logging uses Buffer for message transmision.
 * All strings should be null-terminated!
 */

/* Send logging message from the buffer. Asynchronious.
 *
 * (!) This will lazily initialise logging backend when first called.
 * Returns false when the internal queue is full (does not wait).
 * Anyway, the buffer is taken by this function, so when
 * is_owner=true, the buffer will be deleted!
 */
bool log(Buffer buf);

/* Send logging message from the buffer. Syncronious.
 *
 * It is to be used when RTOS is not running. The call blocks until
 * the data is sent. It is the best to use static Buffer with
 * this function (Buffer::from_static()).
 */
bool log_blocking(Buffer buf);

/* Send logging message from the buffer. Asynchronious. From ISR.
 *
 * This function has zero timeout, because we cannot block in ISR.
 * It won't work (and will return false) if logging has not been
 * initialised yet - this should prevent problems with interrupts
 * fireing before RTOS is started.
 *
 * If a context switch is required, it will set should_yield=true,
 * else it won't change should_yield, so one variable can be used
 * for multiple calls to this (works like logical OR).
 * If a context switch is required, user should (most probably)
 * perform it at the end of an interrupt routine.
 */
bool log_from_isr(Buffer buf, bool &should_yield);

} // namespace logging

