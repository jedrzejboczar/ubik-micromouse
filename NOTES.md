
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
