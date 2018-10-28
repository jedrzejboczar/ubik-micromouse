#pragma once

#include "device_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "freeRTOScpp/task_cpp.h"

class FreeRTOSDriverTask:
    public DriverTask<QueueHandle_t>, public FreeRTOSTask
{
    QueueHandle_t request_queue;
    size_t request_queue_length;
    TickType_t timeout;
public:
    using pQueue_t = QueueHandle_t;

    FreeRTOSDriverTask(
            const char * name, int priority,
            int stack_depth,
            size_t request_queue_length, TickType_t timeout):
        FreeRTOSTask(name, priority, stack_depth),
        request_queue_length(request_queue_length), timeout(timeout) {  }

    // from FreeRTOSTask:
    virtual void task() final { run(); }

    virtual pQueue_t create_queue(size_t queue_length, size_t data_size) override;
    virtual void create_request_queue(size_t data_size) override;
    virtual bool receive_request(void *into_buffer) override;
    virtual bool wait_for_isr() override;
    virtual void send_response(pQueue_t response_queue, const void *from_buffer) override;
};
