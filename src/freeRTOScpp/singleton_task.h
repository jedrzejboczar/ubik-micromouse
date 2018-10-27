#include "task_cpp.h"

/*
 * ACTUALLY IT IS NOT SO NICE
 * There is problem with multiple inheritance and so we have
 * code duplication anyway...
 *
 *
 * Because often we want to use tasks as singletons anyway (to
 * control some hardware resources), this is a convenient class
 * for this purpose.
 *
 * Basically a copy of:
 * https://stackoverflow.com/questions/34519073/inherit-singleton
 *
 * How to derive (we use CRTP):
 *
 * class SomeTask: public FreeRTOSTaskSingleton<SomeTask> {
 *
 *     // we need to declare it as friend class
 *     friend class FreeRTOSTaskSingleton<SomeTask>;
 *
 *     // declare the default constructor to specify reqired parameters
 *     SomeTask(): FreeRTOSTaskSingleton<SomeTask>("Main", 2, 200) { }
 *
 *     virtual void task() override {
 *         vTaskDelay(portMAX_DELAY);
 *     }
 *
 * };
 *
 */

template <typename T>
class FreeRTOSTaskSingleton: public FreeRTOSTask {
protected:
    FreeRTOSTaskSingleton() noexcept = default;
    FreeRTOSTaskSingleton(const FreeRTOSTaskSingleton&) = delete;
    FreeRTOSTaskSingleton& operator=(const FreeRTOSTaskSingleton&) = delete;

    FreeRTOSTaskSingleton(const char * const name, UBaseType_t priority,
            const configSTACK_DEPTH_TYPE stack_depth=configMINIMAL_STACK_SIZE):
        FreeRTOSTask(name, priority, stack_depth)
    {  }

    // to silence base class FreeRTOSTaskSingleton<T> has a non-virtual destructor [-Weffc++]
    virtual ~FreeRTOSTaskSingleton() = default;

public:
    static T& get()
    {
        // Guaranteed to be destroyed.
        // Instantiated on first use.
        // Thread safe in C++11
        static T instance;

        return instance;
    }
};
