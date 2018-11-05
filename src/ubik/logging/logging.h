#pragma once

#include "buffer.h"

namespace logging {

/* Logging uses Msg for message transmision.
 * (!) All strings should be null-terminated.
 */
struct Msg: Buffer {
    // inherit constructors, add default confersion for static methods
    using Buffer::Buffer;
    Msg(Buffer buf): Buffer(buf) {}

    // access the buffer as a string
    char *as_chars() {
        return reinterpret_cast<char *>(data);
    }

    // reimplement static methods to return Msg
    static Msg dynamic(size_t size, bool is_owner=true) {
        return Buffer::dynamic(size, is_owner);
    }

    // how to do these prettier?
    template<size_t size>
    static Msg from_static(uint8_t (&buf)[size]) {
        return Buffer::from_static(buf);
    }
    template<size_t size>
    static Msg from_static(const uint8_t (&buf)[size]) {
        return Buffer::from_static(const_cast<uint8_t (&)[size]>(buf));
    }
    template<size_t size>
    static Msg from_static(char (&buf)[size]) {
        return Buffer::from_static(reinterpret_cast<uint8_t (&)[size]>(buf));
    }
    template<size_t size>
    static Msg from_static(const char (&buf)[size]) {
        return Buffer::from_static(reinterpret_cast<uint8_t (&)[size]>(const_cast<char (&)[size]>(buf)));
    }
};

/*
 * These functions allow to stop the logger task from processing
 * messages from queue.
 * IMPORTANT: You should use log_blocking() while holding the lock,
 * as by using the queue, it may happen that it is full and the
 * messages will be lost.
 * Also the scheduler state (running/suspended) at the moment of
 * locking must be the same as when unlocking.
 */
void lock();
void unlock();

/* Send logging message from the buffer. Asynchronious.
 *
 * (!) This will lazily initialise logging backend when first called.
 * By default does not wait for place in the queue if it is full,
 * but returns false. If wait=true, will wait forever.
 * Anyway, the buffer is taken by this function, so when
 * is_owner=true, the buffer will be deleted!
 */
bool log(Msg msg, bool wait=false);

/* Send logging message from the buffer. Syncronious.
 *
 * It is to be used when RTOS is not running.
 * (!) If the scheduler is running than you MUST first lock().
 * The call blocks until the data is sent. It may be the best to
 * use static Msg with this function (Msg::from_static()).
 */
bool log_blocking(Msg msg);

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
bool log_from_isr(Msg msg, bool &should_yield);


/*
 * Convenient "macros" to easily print formatted logging messages.
 * Although it may seem, that it will take up more space in the
 * final binary, it seems to actually generate smaller binaries
 * then when typing all this code each time by hand, and for sure
 * it is safer (see performance test results after the declarations).
 * (!) One important problem is that -Wformat-security does not work -
 *     user shoudl double check that the string format and arguments are ok.
 *     See:
 *     https://stackoverflow.com/questions/12171132/avoid-warning-in-wrapper-around-printf
 *     and:
 *     https://gcc.gnu.org/onlinedocs/gcc-3.2/gcc/Function-Attributes.html
 *     This could have worked if we used macro, or variadic function.
 *     But as it is a template, we must specify 0 to the attribute:
 *         __attribute__((format(printf, 2, 0)))
 *     and still compiler generates warings.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
#pragma GCC diagnostic ignored "-Wformat-security"
#pragma GCC diagnostic ignored "-Wvla"

// used for 1 or more arguments
template<typename FirstArg, typename... Args>
__attribute__((format(printf, 2, 0)))
// static inline
bool printf(size_t alloc_size, const char *format_str, FirstArg printf_arg1, Args... printf_args) {
    Msg msg = Msg::dynamic(alloc_size);
    int n_written = snprintf(msg.as_chars(), msg.size, format_str, printf_arg1, printf_args...);
    if (n_written < 0) {
        msg.delete_if_owned();
        return false;
    }
    return log(msg);
}
// used for 0 arguments only
template<typename... Args>
bool printf(size_t alloc_size, const char *format_str, Args... printf_args) {
    return printf(alloc_size, "%s", format_str, printf_args...);
}

template<typename FirstArg, typename... Args>
__attribute__((format(printf, 2, 0)))
// static inline
bool printf_blocking(size_t buf_size, const char *format_str, FirstArg printf_arg1, Args... printf_args) {
    uint8_t buf[buf_size];
    Msg msg{buf, buf_size, false};
    int n_written = snprintf(msg.as_chars(), msg.size, format_str, printf_arg1, printf_args...);
    if (n_written < 0)
        return false;
    return log_blocking(msg);
}
// used for 0 arguments only
template<typename... Args>
bool printf_blocking(size_t buf_size, const char *format_str, Args... printf_args) {
    return printf_blocking(buf_size, "%s", format_str, printf_args...);
}

template<typename FirstArg, typename... Args>
__attribute__((format(printf, 3, 0)))
// static inline
bool printf_from_isr(bool &should_yield, size_t alloc_size, const char *format_str, FirstArg printf_arg1, Args... printf_args) {
    Msg msg = Msg::dynamic(alloc_size);
    int n_written = snprintf(msg.as_chars(), msg.size, format_str, printf_arg1, printf_args...);
    if (n_written < 0) {
        msg.delete_if_owned();
        return false;
    }
    return log_from_isr(msg, should_yield);
}
// used for 0 arguments only
template<typename... Args>
bool printf_from_isr(bool &should_yield, size_t alloc_size, const char *format_str, Args... printf_args) {
    return printf_from_isr(should_yield, alloc_size, "%s", format_str, printf_args...);
}

/*
 * We must also provide specialisation for printing without format arguments
 * because the string should be null-terminated.
 */
// template<typename... Args>
// __attribute__((format(printf, 2, 0)))
// // static inline
// bool printf(size_t alloc_size, const char *format_str, Args... printf_args) {


#pragma GCC diagnostic pop

/* Test code:
 * - version 1:
 *     logging::Msg msg;
 *     msg = logging::Msg::dynamic(60);
 *     snprintf(msg.as_chars(), msg.size, "Hello world!");
 *     logging::log(msg);
 *     msg = logging::Msg::dynamic(60);
 *     snprintf(msg.as_chars(), msg.size, "Hello world %d!", 10);
 *     logging::log(msg);
 *     msg = logging::Msg::dynamic(60);
 *     snprintf(msg.as_chars(), msg.size, "Hello world %d %d!", 10, 10);
 *     logging::log(msg);
 *     msg = logging::Msg::dynamic(60);
 *     snprintf(msg.as_chars(), msg.size, "Hello world %d %d %d!", 10, 10, 10);
 *     logging::log(msg);
 * - version 2:
 *     logging::printf(100, "Hello world!");
 *     logging::printf(100, "Hello world %d!", 10);
 *     logging::printf(100, "Hello world %d %d!", 10, 10);
 *     logging::printf(100, "Hello world %d %d %d!", 10, 10, 10);
 *
 * Results:
 * - version 1:
 *     - debug (-Og):
 *           Memory region         Used Size  Region Size  %age Used
 *                      FLASH:       32336 B        64 KB     49.34%
 *                        RAM:       10084 B        20 KB     49.24%
 *              text	   data	    bss	    dec	    hex	filename
 *             32212	    120	   9972	  42304	   a540	ubik
 *     - release (-Os):
 *           Memory region         Used Size  Region Size  %age Used
 *                      FLASH:       18372 B        64 KB     28.03%
 *                        RAM:       10080 B        20 KB     49.22%
 *              text	   data	    bss	    dec	    hex	filename
 *             18256	    116	   9972	  28344	   6eb8	ubik
 * - version 2:
 *     - debug (-Og):
 *           Memory region         Used Size  Region Size  %age Used
 *                      FLASH:       32432 B        64 KB     49.49%
 *                        RAM:       10084 B        20 KB     49.24%
 *              text	   data	    bss	    dec	    hex	filename
 *             32308	    120	   9972	  42400	   a5a0	ubik
 *     - release (-Os):
 *           Memory region         Used Size  Region Size  %age Used
 *                      FLASH:       18352 B        64 KB     28.00%
 *                        RAM:       10080 B        20 KB     49.22%
 *              text	   data	    bss	    dec	    hex	filename
 *             18236	    116	   9972	  28324	   6ea4	ubik
 */



} // namespace logging

