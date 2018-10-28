#include "uart.h"

UART::UART(UART_HandleTypeDef *huart): huart(huart) {

}

void UART::configure() {
    (void) 0;
}

bool UART::start_async(const RequestData_t &data) {
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart,
            // no comments...
            const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(&data)),
            sizeof(RequestData_t));
    return status == HAL_OK;
}

void UART::abort() {
    HAL_UART_Abort(huart);
}

void UART::prepare_response(ResponseData_t &data) {
    data = true;
}

