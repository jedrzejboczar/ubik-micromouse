#pragma once

#include "freeRTOS_cpp/task_cpp.h"
#include "event_groups.h"

#include "macros.h"

namespace supervision {


// common events (bit number)
// the rest of events is for signalization of task being ready
enum Event {
    START = 0, STOP,
    Event_COUNT
};

// sets the bit corresponding to the given task's event
EventBits_t event_task_bit(int task_id);

// sets all bits corresponding to task events (without events START, STOP)
EventBits_t event_all_tasks_bits();


class SupervisedTask;

class SupervisorTask: FreeRTOSTask {
    friend class SupervisedTask;
public:
    static constexpr int priority = configMAX_PRIORITIES;
protected:
    // called inside task() after resource initialization
    virtual void run() = 0;

    // performs events for all tasks
    void start_all(TickType_t timeout=portMAX_DELAY);
    void stop_all(TickType_t timeout=portMAX_DELAY);

    SupervisorTask(const char *name, int stack_size);
private:
    static EventGroupHandle_t system_events;

    virtual void task() final;
    void perform_event(Event event, TickType_t timeout);
};

/*
 * The base class for all the tasks that are supervised.
 * Supervised tasks start only after the system has been
 * started by the supervisor.
 * Each task should call handle_system_events() periodically,
 * because, when system stop has been requested, the supervisor
 * task will wait some time for tasks to stop gracefully
 * and then will force-stop all other tasks.
 *
 */
class SupervisedTask: FreeRTOSTask {
protected:
    // mehtods the same as component hooks in Orocos (no cleanup)
    virtual void configure() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void update() = 0;

    SupervisedTask(const char *name, int priority, int stack_size);

public:
    // void set_priority(int priority);
    static int tasks_count() { return _tasks_count; }
    int id() { return _id; }

private:
    static int _tasks_count;
    int _id;  // 0, 1, 2, ...

    virtual void task() final;
    bool stop_requested();
    void wait_for_start();
    void signal_task_ready();
};


} // namespace Supervision

