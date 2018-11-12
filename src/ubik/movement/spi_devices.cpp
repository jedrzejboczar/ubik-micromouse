#include "spi_devices.h"

#include "stm32f1xx.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include <cstring> // memset

namespace spi {

/*** HARDWARE *****************************************************************/

extern "C" SPI_HandleTypeDef hspi2;
static SPI_HandleTypeDef &hspi = hspi2;
// CS pins
static GPIO_TypeDef *enc_left_cs_port  = ENC2_Cs_GPIO_Port;
static uint16_t      enc_left_cs_pin   = ENC2_Cs_Pin;
static GPIO_TypeDef *enc_right_cs_port = ENC1_Cs_GPIO_Port;
static uint16_t      enc_right_cs_pin  = ENC1_Cs_Pin;
static GPIO_TypeDef *gpioex_cs_port    = GPIO_Exp_Cs_GPIO_Port;
static uint16_t      gpioex_cs_pin     = GPIO_Exp_Cs_Pin;

/******************************************************************************/

static SemaphoreHandle_t mutex = nullptr;

static bool initialise_encoders();
static bool initialise_gpio_expander();
static void lock();
static void unlock();


void initialise() {
    static bool initialised = false;
    if (initialised) return;
    initialised = true;

    mutex = xSemaphoreCreateMutex();
    configASSERT(mutex != nullptr);

    bool ok;
    ok = initialise_encoders();
    configASSERT(ok);
    ok = initialise_gpio_expander();
    configASSERT(ok);
}

EncoderReadings read_encoders() {
    HAL_StatusTypeDef left_status, right_status;
    int timeout_ms = 2;

    // fill the buffer with zeros (this is somehow important, still don't know why)
    uint8_t buffer[6] = {0};

    // get access to SPI
    lock();

    // read left
    HAL_GPIO_WritePin(enc_left_cs_port, enc_left_cs_pin, GPIO_PIN_RESET);
    left_status = HAL_SPI_Receive(&hspi, buffer, 3, timeout_ms);
    HAL_GPIO_WritePin(enc_left_cs_port, enc_left_cs_pin, GPIO_PIN_SET);
    // read right to the next 3 bytes in the buffer
    HAL_GPIO_WritePin(enc_right_cs_port, enc_right_cs_pin, GPIO_PIN_RESET);
    right_status = HAL_SPI_Receive(&hspi, buffer + 3, 3, timeout_ms);
    HAL_GPIO_WritePin(enc_right_cs_port, enc_right_cs_pin, GPIO_PIN_SET);

    // return access to SPI
    unlock();

    // prepare the result
    EncoderReadings results;
    results.valid = left_status == HAL_OK && right_status == HAL_OK;
    if (results.valid) {
        results.left = AS5045Reading::from_buffer(buffer);
        results.right = AS5045Reading::from_buffer(buffer + 3);
    }

    return results;
}

bool update_gpio_expander_pins(uint8_t bits_to_set, uint8_t bits_to_reset) {
    // TODO: update_gpio_expander_pins
    return false;
}

static bool initialise_encoders() {
    bool compensation_finished = false;
    int i = 0, max_attempts = 10;
    while (!compensation_finished && i++ < max_attempts) {
        EncoderReadings results = read_encoders();
        if (results.both_ok())
            compensation_finished = true;
        else
            HAL_Delay(2);
    }
    return compensation_finished;
}

static bool initialise_gpio_expander() {
    // TODO:
    return true;
}

static void lock() {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        // this should always succeed as we wait indefinitelly
        bool taken = xSemaphoreTake(mutex, portMAX_DELAY) == pdPASS;
        configASSERT(taken);
    }
}

static void unlock() {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        // this could only fail if we didn't take the semaphore earlier
        bool could_give = xSemaphoreGive(mutex) == pdPASS;
        configASSERT(could_give);
    }
}



} // namespace spi
