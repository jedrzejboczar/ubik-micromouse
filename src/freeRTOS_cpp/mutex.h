#pragma once

#include "lockable.h"
#include "task.h"
#include "semphr.h"


class Mutex: public Lockable {
public:
    Mutex() {
        handle = xSemaphoreCreateMutex();
        configASSERT(handle != nullptr);
    }
    virtual ~Mutex() {
        vSemaphoreDelete(handle);
    }
    bool take(TickType_t block_time=portMAX_DELAY) {
        return xSemaphoreTake(handle, block_time) == pdTRUE;
    }
    bool give() {
        return xSemaphoreGive(handle) == pdTRUE;
    }
private:
    SemaphoreHandle_t handle;
    // derive constructors
    using Lockable::Lockable;
};


