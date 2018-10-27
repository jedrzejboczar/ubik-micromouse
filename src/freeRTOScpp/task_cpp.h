#pragma once


#include "FreeRTOS.h"
#include "task.h"

class FreeRTOSTask {
public:
    TaskHandle_t task_handle;

    /*
     * Creates the task, if the task was not created,
     * then task_handle will be set to nullptr!
     */
    FreeRTOSTask(const char * const name, UBaseType_t priority,
            const configSTACK_DEPTH_TYPE stack_depth=configMINIMAL_STACK_SIZE)
    {
        BaseType_t result = xTaskCreate(&task_function_wrapper,
                name, stack_depth, this, priority, &task_handle);
        if (result != pdPASS)
            task_handle = nullptr;
    }

    /*
     * Task is not deleted on destruction of this object,
     * which means that it can be used to spawn a task (and forget about it)
     */
    virtual ~FreeRTOSTask() {}

    // delete copy constructor and assign operator
    FreeRTOSTask(FreeRTOSTask const&) = delete;
    void operator =(FreeRTOSTask const&) = delete;

protected:
    /*
     * This is the task function to be implemented,
     * if INCLUDE_vTaskDelete is not defined, then it should never return.
     */
    virtual void task() = 0;

private:
    /*
     * This is a functions that wraps the task function
     * to call abstract member function task().
     * Basicly all tasks created start in this function,
     * and only call task() implemented in deriving class.
     */
    static void task_function_wrapper(void *parm) {
        FreeRTOSTask *task = static_cast<FreeRTOSTask *>(parm);
        task->task();

#if ( INCLUDE_vTaskDelete == 1 )
        vTaskDelete(nullptr);
#else
        configASSERT(0);
#endif
    }
};


