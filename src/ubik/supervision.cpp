#include "supervision.h"

namespace supervision {


#if configUSE_16_BIT_TICKS == 1
constexpr int max_num_events = 8;
#else
constexpr int max_num_events = 24;
#endif

EventGroupHandle_t SupervisorTask::system_events;

EventBits_t event_task_bit(int task_id) {
    return bit(Event_COUNT + task_id);
}

// sets all bits corresponding to task events (without events START, STOP)
EventBits_t event_all_tasks_bits() {
    return ( (1 << SupervisedTask::tasks_count()) - 1 ) << Event_COUNT;
}


// // TODO: make perform_event() force suspension of tasks after timeout!
// TaskHandle_t tasks_by_id[] = { nullptr };

SupervisorTask::SupervisorTask(const char *name, int stack_size):
    FreeRTOSTask(name, priority, stack_size)
{ }

void SupervisorTask::task() {
    system_events = xEventGroupCreate();
    configASSERT(system_events);
    run();
}

void SupervisorTask::perform_event(Event event, TickType_t timeout) {
    // first clear all event bits
    xEventGroupClearBits(system_events,
            START | STOP | event_all_tasks_bits());
    // set the bit corresponding to performed event
    xEventGroupSetBits(system_events, event);
    // wait for tasks to signalize ready,
    xEventGroupWaitBits(system_events, event_all_tasks_bits(),
            pdTRUE, pdTRUE, timeout); // clear the bits, logical AND
}

void SupervisorTask::start_all(TickType_t timeout) {
    perform_event(START, timeout);
}

void SupervisorTask::stop_all(TickType_t timeout) {
    perform_event(STOP, timeout);
}


int SupervisedTask::_tasks_count = 0;

SupervisedTask::SupervisedTask(const char *name, int priority, int stack_size):
    FreeRTOSTask(name, priority, stack_size)
{
    // all supervised tasks have to have lower priority than supervisor
    configASSERT(priority < SupervisorTask::priority);

    _id = _tasks_count;
    configASSERT(_id < (max_num_events - Event::Event_COUNT));

    // tasks_by_id[id()] = task_handle;
    _tasks_count++;
}

// void SupervisedTask::set_priority(int priority) {
//     vTaskPrioritySet(task_handle, priority);
// }

void SupervisedTask::task() {
    configure();
    signal_task_ready();

    wait_for_start();

    start();
    signal_task_ready();

    while(1) {
        if (stop_requested()) {
            stop();
            signal_task_ready();

            wait_for_start();

            start();
            signal_task_ready();
        }
        // execute next "loop"
        update();
    }
}

bool SupervisedTask::stop_requested() {
    // check if the stop event has been requested
    EventBits_t bits = xEventGroupGetBits(SupervisorTask::system_events);
    return (bits & bit(Event::STOP)) != 0;
}

void SupervisedTask::wait_for_start() {
    // wait for next start
    xEventGroupWaitBits(SupervisorTask::system_events, START, pdFALSE, pdTRUE, portMAX_DELAY);
}

void SupervisedTask::signal_task_ready() {
    // set the task's ready bit
    xEventGroupSetBits(SupervisorTask::system_events, event_task_bit(id()));
}

} // namespace Supervision

