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

namespace MCP {
// MCP23S08 registers
static constexpr uint8_t IODIR           = 0x00;
static constexpr uint8_t IPOL            = 0x01;
static constexpr uint8_t GPINTEN         = 0x02;
static constexpr uint8_t DEFVAL          = 0x03;
static constexpr uint8_t INTCON          = 0x04;
static constexpr uint8_t IOCON           = 0x05;
static constexpr uint8_t GPPU            = 0x06;
static constexpr uint8_t INTF            = 0x07;
static constexpr uint8_t INTCAP          = 0x08;
static constexpr uint8_t GPIO            = 0x09;
static constexpr uint8_t OLAT            = 0x0a;
// MCP23S08 SPI contorol bytes
static constexpr uint8_t CTRL_BYTE_WRITE = 0x40;
static constexpr uint8_t CTRL_BYTE_READ  = 0x41;
}

/******************************************************************************/

enum Device { AS5045_ENCODERS, MCP23S08_GPIO_EXPANDER };

static SemaphoreHandle_t mutex = nullptr;

// collection of bits for port
static uint8_t gpioex_port_state = 0;

static bool initialise_encoders();
static bool initialise_gpio_expander();
static bool gpio_expander_write3(uint8_t b1, uint8_t b2, uint8_t b3);
static void configure_spi_mode(Device dev);
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

    // set correct SPI mode
    configure_spi_mode(AS5045_ENCODERS);

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

bool gpio::update_pins(uint8_t bits_to_set, uint8_t bits_to_reset) {
    uint8_t new_state = gpioex_port_state;
    new_state |= bits_to_set;
    new_state &= ~(bits_to_reset);

    bool ok = gpio_expander_write3(MCP::CTRL_BYTE_WRITE, MCP::OLAT, new_state);

    if (ok)
        gpioex_port_state = new_state;

    return ok;
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
    // configure all pins as outputs
    bool ok = gpio_expander_write3(MCP::CTRL_BYTE_WRITE, MCP::IODIR, 0x00);

    // set all pins to LOW
    ok &= gpio::update_pins(0x00, 0xff);
    gpioex_port_state = 0x00;

    return ok;
}

static bool gpio_expander_write3(uint8_t b1, uint8_t b2, uint8_t b3) {
    int timeout_ms = 2;
    bool ok;

    // get access to SPI
    lock();

    // set correct SPI mode
    configure_spi_mode(MCP23S08_GPIO_EXPANDER);

    uint8_t buffer[] = {b1, b2, b3};
    HAL_GPIO_WritePin(gpioex_cs_port, gpioex_cs_pin, GPIO_PIN_RESET);
    ok = HAL_SPI_Transmit(&hspi, buffer, sizeof(buffer), timeout_ms) == HAL_OK;
    HAL_GPIO_WritePin(gpioex_cs_port, gpioex_cs_pin, GPIO_PIN_SET);

    // return access to SPI
    unlock();

    return ok;
}

static void configure_spi_mode(Device dev) {
    // for AS5045_ENCODERS we need MSB first, CLK pol high, CLK phase 1st edge
    // (see as5045.h)
    // for MCP23S08 we need MSB first, CLK pol low, CLK phase 1st edge
    switch (dev) {
        case AS5045_ENCODERS:
            {
                uint32_t reg = hspi.Instance->CR1;
                reg |= SPI_CR1_CPOL; // idle high
                reg &= ~SPI_CR1_CPHA; // 1st edge
                hspi.Instance->CR1 = reg;
                break;
            }
        case MCP23S08_GPIO_EXPANDER:
            {
                uint32_t reg = hspi.Instance->CR1;
                reg &= ~SPI_CR1_CPOL; // idle low
                reg &= ~SPI_CR1_CPHA; // 1st edge
                hspi.Instance->CR1 = reg;
                break;
            }
            break;
    }
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
