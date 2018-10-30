#pragma once

#include "task_cpp.h"

/*
 * Template for spawning tasks from lambda functions.
 *
 * Example:
 *   auto my_lambda = [this]{
 *       this->do_somehing();
 *   };
 *   LambdaTask<decltype(my_lambda)> my_task(my_lambda, uxTaskPriorityGet(nullptr));
 *
 */
template<typename Lambda>
class LambdaTask: FreeRTOSTask {
public:
    LambdaTask(Lambda lambda, UBaseType_t priority,
            const configSTACK_DEPTH_TYPE stack_depth=configMINIMAL_STACK_SIZE):
        FreeRTOSTask("Lambda", priority, stack_depth),
        lambda(lambda),
        // constructor is called in the task we want to notify so we get the handle
        caller_task(xTaskGetCurrentTaskHandle())
    { }
    /*
     * Convenient function for waiting for lambda tasks created.
     * notification_val should be 1 at the end (if nothing else notified this task
     * and notification value was 0 at the beggining)
     */
    static uint32_t wait_for(int how_many=1) {
        uint32_t notification_val;
        for (int i = 0; i < how_many; i++)
            notification_val = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        return notification_val;
    }
private:
    Lambda lambda;
    TaskHandle_t caller_task;
    // calls the lambda, notifies the caller and deletes itself (FreeRTOSTask)
    virtual void task() override {
        lambda();
        xTaskNotifyGive(caller_task);
    }
};
