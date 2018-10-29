#include "freertos_driver.h"


// void FreeRTOSDriverTask::task() {
//     run();
// }

FreeRTOSDriverTask::FreeRTOSDriverTask(
        const char * name, int priority, int stack_depth,
        size_t request_queue_length, TickType_t timeout):
    FreeRTOSTask(name, priority, stack_depth),
    request_queue_length(request_queue_length), timeout(timeout), runner(nullptr)
{ }

FreeRTOSDriverTask::pQueue_t FreeRTOSDriverTask::create_queue(
        size_t queue_length, size_t data_size)
{
    return xQueueCreate(queue_length, data_size);
}

void FreeRTOSDriverTask::create_request_queue(size_t data_size) {
    request_queue = create_queue(request_queue_length, data_size);
    configASSERT(request_queue);
}

bool FreeRTOSDriverTask::receive_request(void *into_buffer) {
    BaseType_t result = xQueueReceive(request_queue,
            into_buffer, portMAX_DELAY); // TODO: how to specify the delay!!!
    return result == pdTRUE;
}

bool FreeRTOSDriverTask::wait_for_isr() {
    uint32_t notification_value = ulTaskNotifyTake(pdTRUE,
            portMAX_DELAY); // TODO: how to specify the delay!!!
    return notification_value != 0;
}

void FreeRTOSDriverTask::send_response(pQueue_t response_queue, const void *from_buffer) {
    BaseType_t result = xQueueSend(response_queue,
            from_buffer, portMAX_DELAY); // TODO: how to specify the delay!!!
    (void) result; // the driver doesn't care, we could try multiple times if needed
}

void FreeRTOSDriverTask::set_task_runner(TaskRunner *runner) {
    this->runner = runner;
}

void FreeRTOSDriverTask::task() {
    configASSERT(runner != nullptr);
    runner->run();
}
