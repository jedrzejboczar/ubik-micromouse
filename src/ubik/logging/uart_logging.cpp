#include "logging.h"

#include <cstring>

#include "stm32f1xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "ubik/timing.h"


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
constexpr size_t queue_length = 3;
QueueHandle_t log_queue = nullptr;
TaskHandle_t log_task = nullptr;
SemaphoreHandle_t log_guard = nullptr;

// stats
uint32_t logs_lost;
uint32_t logs_lost_from_uart_errors;
uint32_t logs_lost_from_notification_timeouts;
uint32_t logs_lost_from_isr;
uint32_t last_uart_transmission_time_us;

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
    // queue and mutex
    log_queue = xQueueCreate(queue_length, sizeof(Msg));
    log_guard = xSemaphoreCreateMutex();
    configASSERT(log_queue != nullptr);
    configASSERT(log_guard != nullptr);
    // task
    bool task_created = xTaskCreate(logger_task, "Logging",
            task_stack_size, nullptr,
            task_priority, &log_task) == pdPASS;
    configASSERT(task_created);
}


void lock() {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        // this should always succeed as we wait indefinitelly
        bool taken = xSemaphoreTake(log_guard, portMAX_DELAY) == pdPASS;
        configASSERT(taken);
    }
}

void unlock() {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        // this could only fail if we didn't take the semaphore earlier
        bool could_give = xSemaphoreGive(log_guard) == pdPASS;
        configASSERT(could_give);
    }
}

bool log(Msg msg, bool wait) {
    // lazy initialization of the module
    if (log_queue == nullptr)
        initialise();
    BaseType_t timeout = wait ? portMAX_DELAY : 0;
    bool result = xQueueSend(log_queue, &msg, timeout) == pdPASS;
    // if the call didn't succeed, we have to free memory here
    if (!result)
        msg.delete_if_owned();
    // stats
    if (!result) logs_lost++;
    return result;
}

bool log_from_isr(Msg msg, bool &should_yield) {
    bool result = false;
    // ignore call when not yet initialsed
    if (log_queue != nullptr) {
        result = xQueueSendFromISR(log_queue, &msg,
                // will set should_yield if needed, else not change
                reinterpret_cast<BaseType_t*>(&should_yield)) == pdPASS;
        if (!result)
            msg.delete_if_owned();
    }
    // stats
    if (!result) {
        logs_lost++;
        logs_lost_from_isr++;
    }
    return result;
}

bool log_blocking(Msg msg) {
    bool result = false;

    // lazy initialization of the module
    if (log_queue == nullptr)
        initialise();

    // send only string part of the buffer
    size_t string_size = strnlen(msg.as_chars(), msg.size - 1);
    uint32_t timeout = millis_per_size(log_uart.Init.BaudRate, string_size);

    // measure transmission time
    cycles_counter::reset();
    cycles_counter::start();

    result = HAL_UART_Transmit(&log_uart, msg.data, string_size, timeout) == HAL_OK;

    // save time on success
    cycles_counter::stop();
    if (result) {
        last_uart_transmission_time_us = cycles_counter::get_us();
    }

    msg.delete_if_owned();

    // stats
    if (!result) {
        logs_lost++;
        logs_lost_from_isr++;
    }

    return result;
}

/******************************************************************************/

void logger_task(void *) {
    Msg msg;

    while (1) {
        if (xQueueReceive(log_queue, &msg, portMAX_DELAY) != pdPASS)
            continue;

        // send only string part of the buffer
        size_t string_size = strnlen(msg.as_chars(), msg.size - 1);

        // lock UART
        lock();

        // measure transmision time
        cycles_counter::reset();
        cycles_counter::start();

        // start write operation using DMA
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
            } else {
                // success, got notification from UART interrupt
                cycles_counter::stop();
                last_uart_transmission_time_us = cycles_counter::get_us();
            }
        }

        // stop using UART
        unlock();

        // delete memory if required
        msg.delete_if_owned();
    }
}

/*
 * Because I'm stupid and i thought that baud rate is in bytes/sec.
 * It is in BITS/SEC, and as UART has 1 parity bit and 1 stop bit
 * we send 10 bits per each byte.
 * So in fact our bytes/sec is 10 times lower than baud rate.
 *
 * Timings that I've done while debugging beacuse I was so stupid:
 * ( -Og, sysclk=72MHz, set baud rate=115200, 100 measurements )
 *
 * HAL_UART_Transmit
 *        --time [us]---   -baud [bits/s]--
 * size       mean   std       mean     std
 *    1      92.85  2.20   10775.52  250.93
 *   22    1914.00  0.00   11494.25    0.00
 *   23    2000.00  0.00   11500.00    0.00
 *   36    3129.00  0.00   11505.27    0.00
 *   44    3817.75  2.60   11525.12    7.84
 *   45    3907.00  0.00   11517.79    0.00
 *  186   16123.00  0.00   11536.31    0.00
 *  191   16559.00  0.00   11534.51    0.00
 *  192   16644.00  0.00   11535.69    0.00
 *  193   16737.71  0.45   11530.85    0.31
 *  194   16822.46  0.50   11532.20    0.34
 *
 * HAL_UART_Transmit
 *        --time [us]---   -baud [bits/s]--
 * size       mean   std       mean     std
 *   44    3833.32  2.70   11478.31    8.08
 *  186   16139.00  0.00   11524.88    0.00
 *  192   16658.00  0.00   11525.99    0.00
 *  194   16831.33  0.47   11526.12    0.32
 *  195   16918.00  0.00   11526.19    0.00
 *  196   17004.75  1.46   11526.19    0.98
 *  197   17092.00  1.41   11525.86    0.95
 */
static size_t millis_per_size(size_t uart_baudrate, size_t size) {
    constexpr float margin = 0.3;
    float ms_per_bit = 1000.0 / uart_baudrate;
    float ms_per_byte = ms_per_bit * 10;
    // add +1 to always round result up
    return ms_per_byte * size * (1 + margin) + 1;
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
