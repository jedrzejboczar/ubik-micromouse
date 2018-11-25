#include <cstddef>

#include "FreeRTOS.h"

namespace logging {

static constexpr size_t TASK_PRIORITY = 1;
static constexpr size_t QUEUE_LENGTH = 5;
static constexpr size_t TASK_STACK_SIZE = configMINIMAL_STACK_SIZE;

} // namespace logging
