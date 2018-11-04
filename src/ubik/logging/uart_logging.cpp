#include "logging.h"

#include <cstring>

#include "stm32f1xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// needed in assumptions about freertos for conversion to bool
static_assert(pdTRUE == true, "FreeRTOS pdTRUE was assumed to be equal to 'true'");
static_assert(pdTRUE == 1, "FreeRTOS pdTRUE was assumed to be equal to '1'");
static_assert(pdPASS == true, "FreeRTOS pdPASS was assumed to be equal to 'true'");
static_assert(pdFALSE == false, "FreeRTOS pdFALSE was assumed to be equal to 'false'");
static_assert(pdFALSE == 0, "FreeRTOS pdFALSE was assumed to be equal to '0'");


/*
 * Using STM32CubeMX code generation, so we assume that UART is properly
 * configured, with DMA and an interrupt.
 */
extern UART_HandleTypeDef huart1;

namespace logging {

static UART_HandleTypeDef &log_uart = huart1;

/*
 * The whole logging interface is lazily initialized on the first
 * call to log(). We can allow this non-realtimeness, as logging is
 * generally for debugging anyway (or at least we call log() to write
 * something when system starts).
 */
constexpr size_t task_priority = 1;
constexpr size_t task_stack_size = configMINIMAL_STACK_SIZE;
constexpr size_t queue_length = 10;
QueueHandle_t log_queue = nullptr;
TaskHandle_t log_task = nullptr;

// stats
uint32_t logs_lost;
uint32_t logs_lost_from_uart_errors;
uint32_t logs_lost_from_notification_timeouts;
uint32_t logs_lost_from_isr;

// task entry function
void logger_task(void *);
// local functions
static size_t millis_per_size(size_t uart_baudrate, size_t size);
static void notify_from_isr(bool &should_yield);

/******************************************************************************/

void initialise() {
    // ignore subsequent calls
    if (log_queue != nullptr)
        return;
    log_queue = xQueueCreate(queue_length, sizeof(Msg));
    configASSERT(log_queue != nullptr);
    bool task_created = xTaskCreate(logger_task, "Logging",
            task_stack_size, nullptr,
            task_priority, &log_task) == pdPASS;
    configASSERT(task_created);
}

bool log(Msg msg) {
    // lazy initialization of the module
    if (log_queue == nullptr)
        initialise();
    bool result = xQueueSend(log_queue, &msg, 0) == pdPASS;
    // if the call didn't succeed, we have to free memory here
    if (!result)
        msg.delete_if_owned();
    if (!result) logs_lost++;
    return result;
}

bool log_from_isr(Msg msg, bool &should_yield) {
    // ignore call when not yet initialsed
    if (log_queue == nullptr) {
        logs_lost_from_isr++;
        return false;
    }
    bool result = xQueueSendFromISR(log_queue, &msg,
            // will set should_yield if needed, else not change
            reinterpret_cast<BaseType_t*>(&should_yield)) == pdPASS;
    if (!result)
        msg.delete_if_owned();
    if (!result) {
        logs_lost++;
        logs_lost_from_isr++;
    }
    return result;
}

bool log_blocking(Msg msg) {
    size_t string_size = strlen(reinterpret_cast<char *>(msg.data));
    uint32_t timeout = millis_per_size(log_uart.Init.BaudRate, string_size);
    bool result = HAL_UART_Transmit(&log_uart, msg.data, msg.size, timeout) == HAL_OK;
    msg.delete_if_owned();
    return result;
}

/******************************************************************************/

void logger_task(void *) {
    Msg msg;

    while (1) {
        if (xQueueReceive(log_queue, &msg, portMAX_DELAY) != pdPASS)
            continue;

        size_t string_size = strlen(reinterpret_cast<char *>(msg.data));

        if (HAL_UART_Transmit_DMA(&log_uart, msg.data, string_size) != HAL_OK) {
            logs_lost_from_uart_errors++;
            // of cource it could have happend that we got HAL_BUSY, but
            // it would mean that our internal waiting for transmision complete
            // interrupt has failed so we'd rather clean up and continue
            HAL_UART_Abort(&log_uart);
        } else {
            // for simplicity assume that huart.Init is consistent with register values
            size_t ticks_to_wait = pdMS_TO_TICKS(millis_per_size(log_uart.Init.BaudRate, string_size));

            if (ulTaskNotifyTake(pdTRUE, ticks_to_wait) == 0) {
                logs_lost_from_notification_timeouts++;
                HAL_UART_Abort(&log_uart);
            };
        }

        // delete memory if required
        msg.delete_if_owned();
    }
}

// baudrate [bytes/sec], size - number of bytes to be sent
static size_t millis_per_size(size_t uart_baudrate, size_t size) {
    constexpr float margin = 0.3;
    float ms_per_byte = 1000.0 / uart_baudrate;
    // TODO: check how long it takes and try to better estimate time!
    return pdMS_TO_TICKS(ms_per_byte) * size * (1 + margin)      /*remove this:*/+ 10;
}

static void notify_from_isr(bool &should_yield) {
    vTaskNotifyGiveFromISR(log_task, reinterpret_cast<BaseType_t*>(&should_yield));
}

// ubik has only one UART used, so just declare it here
extern "C"
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    bool should_yield = false;

    if (huart->Instance == log_uart.Instance)
        notify_from_isr(should_yield);

    portYIELD_FROM_ISR(should_yield);
}



} // namespace logging
