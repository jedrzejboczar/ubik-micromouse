/*
 * Implementation of microcontroller interrupts,
 * either directly, or using STM32HAL Callback functions.
 *
 * As each callback may be needed for different modules,
 * having all of them implemented in one place is probably
 * the best way to handle this problem. Each implementation
 * should just call a handle function from given module.
 */



extern "C" {

// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
// }
//
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
// }

}

