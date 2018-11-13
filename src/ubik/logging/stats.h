#pragma once

#include "logging.h"

namespace logging {

/*
 * This function is mainly for debugging and should be used with
 * caution, as it locks the whole logging framework for quite a
 * while, and also it requires a lot of memory (from FreeRTOS heap).
 */
void print_stats();


/*
 * Task that collects and prints system information.
 * As everything here, use for debugging only.
 * As an argument the number of ticks specifying the delay between
 * subsequent loops should be passed. It has to be cast to void*.
 */
void system_monitor_task(void *ticks_to_wait_as_ptr);

} // namespace logging

