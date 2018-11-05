#pragma once

#include "logging.h"

namespace logging {

/*
 * This function is mainly for debugging and should be used with
 * caution, as it locks the whole logging framework for quite a
 * while, and also it requires a lot of memory (from FreeRTOS heap).
 */
void print_stats();

} // namespace logging

