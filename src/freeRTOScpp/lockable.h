#pragma once

/*
 * Implementation from:
 * https://github.com/richard-damon/FreeRTOScpp
 */

#include "FreeRTOS.h"


class Lockable {
public:
    Lockable() {}
    virtual ~Lockable() {}

    // non-copyable
    Lockable(Lockable const&) = delete;
    void operator=(Lockable const&) = delete;

    virtual bool take(TickType_t block_time) = 0;
    virtual bool give() = 0;
};

class Lock {
public:
    Lock(Lockable& to_be_locked, bool as_locked=true, TickType_t block_time=portMAX_DELAY):
        lockable(to_be_locked), lock_count(0)
    {
        if (as_locked)
            lock(block_time);
    }
    virtual ~Lock() {
        if (lock_count > 0)
            lockable.give();
        lock_count = 0;
    }

    bool is_locked() const { return lock_count > 0; }
    bool lock(TickType_t block_time=portMAX_DELAY) {
        if ((lock_count > 0) || (lockable.take(block_time))) {
            lock_count++;
            return true;
        }
        return false;
    }
    void unlock() {
        if (lock_count > 0) { // ignore extra unlocks
            lock_count--;
            if (lock_count == 0)
                lockable.give();
        }
    }
private:
    Lockable& lockable;
    int	lock_count;
};



