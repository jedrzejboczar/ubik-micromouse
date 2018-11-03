#include "logging.h"

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

namespace logging {

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

/*
 * Using STM32CubeMX code generation, so we assume that UART is properly
 * configured, with DMA and an interrupt.
 */
extern UART_HandleTypeDef huart1;
static UART_HandleTypeDef &log_uart = huart1;

uint32_t logs_lost;
uint32_t logs_lost_from_uart_errors;
uint32_t logs_lost_from_notification_timeouts;
uint32_t logs_lost_from_isr;

// task entry function
void logger_task(void *);

/******************************************************************************/

void initialise() {
    // ignore subsequent calls
    if (log_queue != nullptr)
        return;
    log_queue = xQueueCreate(queue_length, sizeof(Buffer));
    configASSERT(log_queue != nullptr);
    configASSERT(
            xTaskCreate(logger_task, "Logging",
                task_stack_size, nullptr,
                task_priority, &log_task) == pdPASS
            );
}

bool log(Buffer buf) {
    // lazy initialization of the module
    if (log_queue == nullptr)
        initialise();
    bool result = xQueueSend(log_queue, &buf, 0) == pdPASS;
    if (!result) logs_lost++;
    return result;
}

bool log_from_isr(Buffer buf, bool &should_yield) {
    // ignore call when not yet initialsed
    if (log_queue == nullptr) {
        logs_lost_from_isr++;
        return false;
    }
    bool result = xQueueSendFromISR(log_queue, &buf,
            // will set should_yield if needed, else not change
            reinterpret_cast<BaseType_t*>(&should_yield)) == pdPASS;
    if (!result) {
        logs_lost++;
        logs_lost_from_isr++;
    }
    return result;
}

/******************************************************************************/

// baudrate [bytes/sec], size - number of bytes to be sent
static size_t ticks_per_size(size_t uart_baudrate, size_t size) {
    constexpr float margin = 0.3;
    float ms_per_byte = 1000.0 / uart_baudrate;
    return pdMS_TO_TICKS(ms_per_byte) * size * (1 + margin);
}

static void notify_from_isr(bool &should_yield) {
    vTaskNotifyGiveFromISR(log_task, reinterpret_cast<BaseType_t*>(&should_yield));
}

void logger_task(void *) {
    Buffer buf;

    while (1) {
        if (xQueueReceive(log_queue, &buf, portMAX_DELAY) != pdPASS)
            continue;

        if (HAL_UART_Transmit_DMA(&log_uart, buf.data, buf.size) != HAL_OK) {
            logs_lost_from_uart_errors++;
            // of cource it could have happend that we got HAL_BUSY, but
            // it would mean that our internal waiting for transmision complete
            // interrupt has failed so we'd rather clean up and continue
            HAL_UART_Abort(&log_uart);
            continue;
        }

        // for simplicity assume that huart.Init is consistent with register values
        size_t ticks_to_wait = ticks_per_size(log_uart.Init.BaudRate, buf.size);

        if (ulTaskNotifyTake(pdTRUE, ticks_to_wait) == 0) {
            logs_lost_from_notification_timeouts++;
            HAL_UART_Abort(&log_uart);
            continue;
        };
    }
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
