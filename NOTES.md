
# Bluetooth

- to use the module in normal mode, "Key" pin (text on module is "CE/CLR") should be HIGH; LOW states makes the module enter programming mode

# Timers

How they were used in previous project (ubikCode/):

- TIM2 - timing of distance sensors delays (interrupt)
    prescaler configured for 1us ticks, interrupt delay time set by period

- TIM3 - motors PWM control signal generation
    prescaler 2 -> 24MHz, period 999 -> 24kHz, PWM from 0 to 999 (1000 states)

- TIM4 - interrupt for encoder readings and motor PID regulation loop
    configured for different interrupt frequencies (last 2kHz)

How to use them here:

- TIM3 - motors PWM control, but with finer resolution
    N states - 7200
    no prescaler
    this gives fpwm = fclk / (period+1) = 72e6 / 7.2e3 = 10e3 -> 10kHz

- TIM2 - again for microsecond non-blocking delays for distance sensors measurements

# SPI (encoders + gpio-expander)

SPI has no DMA available.
- reading 3 bytes with HAL_SPI_Receive() takes about 51 us
- reading 3 bytes with a simple implementation on registers takes about 44 us
- reading 3 bytes with HAL_SPI_Receive_IT() takes about 51 us, but have about 18 us of time for other things


# DMA Channels

- DMA1 Channel1 - ADC1      - distance sensor measurements - medium priority
- DMA1 Channel4 - USART1 TX - logging TX                   - low priority
- DMA1 Channel5 - USART1 RX - logging RX                   - low priority
- DMA1 Channel6 - I2C1 TX   - IMU TX                       - high priority, because of I2C1-TIM3 conflict
- DMA1 Channel7 - I2C1 RX   - IMU RX                       - high priority, because of I2C1-TIM3 conflict
