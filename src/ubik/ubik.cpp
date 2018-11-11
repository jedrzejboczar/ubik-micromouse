#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ubik/logging/logging.h"
#include "ubik/logging/stats.h"

void run();
extern "C" void extern_main(void) { run(); }

#include "timing.h"

void stats_task(void *) {
    auto last_start = xTaskGetTickCount();

    while(1) {
        cycles_counter::reset();
        cycles_counter::start();
        logging::print_stats();
        cycles_counter::stop();
        logging::printf(50, "=== Printing stats took %d us ===\n", cycles_counter::get_us());
        vTaskDelayUntil(&last_start, pdMS_TO_TICKS(5000));
    }
}

void dummy_task(void *number_param) {
    int counter = 0;
    uintptr_t number = reinterpret_cast<uintptr_t>(number_param);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));

        logging::printf(100, "This is dummy task %d, counter = %d\n", number, counter++);
    }
}

#include "ubik/movement/driver.h"
#include "ubik/movement/pid.h"
#include "ubik/movement/as5045.h"


// PID vel_pid;

extern TIM_HandleTypeDef htim4;
extern "C" void callback_timer_period_elapsed(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim4.Instance) {
        // read encoders
        // get next PID output
        // set motors pulse
    }
}


void wait_ns(uint32_t ns) {
    cycles_counter::reset();
    cycles_counter::start();
    while (cycles_counter::get_ns() < ns);
    cycles_counter::stop();
}


extern SPI_HandleTypeDef hspi2;
void my_spi_read() {
    // read 18 bits each one sampled 10 times
    bool normal[18] = {0};
    bool so_oversampled[18 * 10] = {0};

    HAL_SPI_DeInit(&hspi2);

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = SPI_Clk_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // do it exactly like on these diagrams
    // set SCK high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

    // CS low
    HAL_GPIO_WritePin(ENC1_Cs_GPIO_Port, ENC1_Cs_Pin, GPIO_PIN_RESET);

    // wait t_CLK_FE then toggle SCK and wait T/2
    wait_ns(500 + 200);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
    wait_ns(500 + 200);

    // now wait at least T/2 between subsequent SCK toggles
    for (int i = 0; i < 18; i++) {

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
        wait_ns(500 + 200);

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
        // read the data
        uint32_t val = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
        wait_ns(500 + 200);

        normal[i] = val != 0;
    }

    // CS high
    HAL_GPIO_WritePin(ENC1_Cs_GPIO_Port, ENC1_Cs_Pin, GPIO_PIN_SET);


    int angle =

            ((normal[0] ? 1 : 0)  << (11 - 0) ) |
            ((normal[1] ? 1 : 0)  << (11 - 1) ) |
            ((normal[2] ? 1 : 0)  << (11 - 2) ) |
            ((normal[3] ? 1 : 0)  << (11 - 3) ) |
            ((normal[4] ? 1 : 0)  << (11 - 4) ) |
            ((normal[5] ? 1 : 0)  << (11 - 5) ) |
            ((normal[6] ? 1 : 0)  << (11 - 6) ) |
            ((normal[7] ? 1 : 0)  << (11 - 7) ) |
            ((normal[8] ? 1 : 0)  << (11 - 8) ) |
            ((normal[9] ? 1 : 0)  << (11 - 9) ) |
            ((normal[10] ? 1 : 0) << (11 - 10)) |
            ((normal[11] ? 1 : 0) << (11 - 11))
        ;

    logging::printf_blocking(200, "[dio] bits: %d%d%d%d%d%d%d%d%d%d%d%d %d %d %d %d %d   %d   angle=%d\n",
            normal[0],
            normal[1],
            normal[2],
            normal[3],
            normal[4],
            normal[5],
            normal[6],
            normal[7],
            normal[8],
            normal[9],
            normal[10],
            normal[11],
            normal[12],
            normal[13],
            normal[14],
            normal[15],
            normal[16],
            normal[17],
            angle
            );
    // logging::printf_blocking(200, "[dio] bits: 0%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d\n",
    //         normal[0],
    //         normal[1],
    //         normal[2],
    //         normal[3],
    //         normal[4],
    //         normal[5],
    //         normal[6],
    //         normal[7],
    //         normal[8],
    //         normal[9],
    //         normal[10],
    //         normal[11],
    //         normal[12],
    //         normal[13],
    //         normal[14],
    //         normal[15],
    //         normal[16],
    //         normal[17]
    //         );
}


int bit_n(uint8_t *data, int n) {
    return (data[n / 8] & (1 << (n % 8))) != 0;
}

static inline void print_bits(const void *memory, int start_bit_num, int n_bits, char *output) {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	int is_big_endian = 1;
#elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	int is_big_endian = 0;
#endif
    int output_iter = 0;

    const unsigned char * const bytes = (const unsigned char *) memory;
	int byte_n = 0, curr_bit_n = start_bit_num;
    // output[output_iter++] = '[';
    // output[output_iter++] = ' ';
	for (size_t i = 0; i < n_bits; i++) {
		int bit_val;
		if (is_big_endian)
			bit_val = (bytes[byte_n] & (1 << curr_bit_n)) != 0;
		else
			bit_val = (bytes[byte_n] & (1 << (7-curr_bit_n))) != 0;
        output[output_iter++] = bit_val ? '1' : '0';
		curr_bit_n++;
		if (curr_bit_n >= 8) {
			curr_bit_n = 0;
			byte_n++;
            // output[output_iter++] = ' ';
		}
	}
    // output[output_iter++] = ']';
}



void spi_read() {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = SPI_Clk_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    // hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    HAL_SPI_Init(&hspi2);

    uint8_t data[4] = {0};

    HAL_GPIO_WritePin(ENC1_Cs_GPIO_Port, ENC1_Cs_Pin, GPIO_PIN_RESET);
    wait_ns(500 + 200);
    auto status = HAL_SPI_Receive(&hspi2, data, 4, 100);
    if (status != HAL_OK)
        logging::printf_blocking(100, "status was = %d\n", static_cast<int>(status));
    HAL_GPIO_WritePin(ENC1_Cs_GPIO_Port, ENC1_Cs_Pin, GPIO_PIN_SET);

    int angle = ( (((uint16_t) data[0]) << 4) & 0xff0 ) | ((data[1] & 0xf0 ) >> 4);

    char bits[100] = {0};
    print_bits(data, 0, 24, bits);
    logging::printf_blocking(200, "[spi] bits: %s\n", bits);

    // logging::printf_blocking(200, "[spi] bits: %d%d%d%d%d%d%d%d%d%d%d%d %d %d %d %d %d   %d   angle=%d\n",
    //
    //         bit_n(data, 0),
    //         bit_n(data, 1),
    //         bit_n(data, 2),
    //         bit_n(data, 3),
    //         bit_n(data, 4),
    //         bit_n(data, 5),
    //         bit_n(data, 6),
    //         bit_n(data, 7),
    //         bit_n(data, 8),
    //         bit_n(data, 9),
    //         bit_n(data, 10),
    //         bit_n(data, 11),
    //         bit_n(data, 12),
    //         bit_n(data, 13),
    //         bit_n(data, 14),
    //         bit_n(data, 15),
    //         bit_n(data, 16),
    //         bit_n(data, 17),
    //
    //         angle
    //         );
    //
}


void run() {
    logging::printf_blocking(100, "\n===========================================\n");
    logging::printf_blocking(100, "Initialising system...\n");

    while (1) {
        my_spi_read();
        // spi_read();




        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = SPI_Clk_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        HAL_SPI_Init(&hspi2);

        uint8_t buf[4];
        // right
        {
            HAL_GPIO_WritePin(ENC1_Cs_GPIO_Port, ENC1_Cs_Pin, GPIO_PIN_RESET);
            // HAL_Delay(1);
            auto status = HAL_SPI_Receive(&hspi2, buf, 3, 10);
            if (status != HAL_OK)
                logging::printf_blocking(100, "status was = %d\n", static_cast<int>(status));
            HAL_GPIO_WritePin(ENC1_Cs_GPIO_Port, ENC1_Cs_Pin, GPIO_PIN_SET);
        }
        AS5045Reading right = AS5045Reading::from_buffer(buf);


        logging::printf_blocking(200, "[spi] bits: %d%d%d%d%d%d%d%d%d%d%d%d %d %d %d %d %d   %d   angle=%d\n",

                bit_n(buf, 0),
                bit_n(buf, 1),
                bit_n(buf, 2),
                bit_n(buf, 3),
                bit_n(buf, 4),
                bit_n(buf, 5),
                bit_n(buf, 6),
                bit_n(buf, 7),
                bit_n(buf, 8),
                bit_n(buf, 9),
                bit_n(buf, 10),
                bit_n(buf, 11),
                bit_n(buf, 12),
                bit_n(buf, 13),
                bit_n(buf, 14),
                bit_n(buf, 15),
                bit_n(buf, 16),
                bit_n(buf, 17),

                right.angle
                    );


        HAL_Delay(50);
    }

    movement::driver::initialise();
    // movement::driver::set_direction(movement::FORWARDS, movement::BACKWARDS);
    movement::driver::set_direction(movement::FORWARDS, movement::FORWARDS);
    movement::driver::set_enabled(true);

    // logging::printf_blocking(50, "max_pulse = %d\n", movement::driver::max_pulse());
    // float max_ratio = .3;
    // int pulse = 0;
    //
    // for (int i = 0; i < 1000; i++) {
    //     pulse = i/1000.0 * max_ratio * movement::driver::max_pulse();
    //     movement::driver::set_pulse(pulse, pulse);
    //     if (i % 10 == 0)
    //         logging::printf_blocking(50, "pulse = %d\n", pulse);
    //     HAL_Delay(3);
    // }
    // for (int i = 1000; i > 0; i--) {
    //     pulse = i/1000.0 * max_ratio * movement::driver::max_pulse();
    //     movement::driver::set_pulse(pulse, pulse);
    //     if (i % 10 == 0)
    //         logging::printf_blocking(50, "pulse = %d\n", pulse);
    //     HAL_Delay(3);
    // }

    movement::driver::set_enabled(false);

    while (1) {

        // extern SPI_HandleTypeDef hspi2;
        // uint8_t reading[10];
        //
        // HAL_GPIO_WritePin(ENC2_Cs_GPIO_Port, ENC2_Cs_Pin, GPIO_PIN_RESET);
        // auto status = HAL_SPI_Receive(&hspi2, reading, 6, 10);
        // if (status != HAL_OK)
        //     logging::printf_blocking(100, "status was = %d\n", static_cast<int>(status));
        // HAL_GPIO_WritePin(ENC2_Cs_GPIO_Port, ENC2_Cs_Pin, GPIO_PIN_SET);
        //
        // uint8_t reading2[10];
        //
        //
        // cycles_counter::reset();
        // cycles_counter::start();
        //
        // HAL_GPIO_WritePin(ENC1_Cs_GPIO_Port, ENC1_Cs_Pin, GPIO_PIN_RESET);
        // auto status2 = HAL_SPI_Receive(&hspi2, reading2, 6, 10);
        // if (status2 != HAL_OK)
        //     logging::printf_blocking(100, "status2 was = %d\n", static_cast<int>(status2));
        // HAL_GPIO_WritePin(ENC1_Cs_GPIO_Port, ENC1_Cs_Pin, GPIO_PIN_SET);
        //
        // cycles_counter::stop();
        // logging::printf_blocking(100, "reading 2 took %d us\n", cycles_counter::get_us());
        //
        // int angle = (((uint16_t) reading[0]) << 4) | ((reading[1] & 0xf0 ) >> 4);
        // int angle2 = (((uint16_t) reading2[0]) << 4) | ((reading2[1] & 0xf0 ) >> 4);
        //
        // logging::printf_blocking(200, "angle = %6d, angle2 = %6d\n", angle, angle2);
        //
        //
        // logging::printf_blocking(200, "[ %02x %02x %02x %02x %02x %02x ]\n",
        //         reading2[0], reading2[1], reading2[2], reading2[3], reading2[4], reading2[5]);
        //

        extern SPI_HandleTypeDef hspi2;
        uint8_t buffer1[10];
        uint8_t buffer2[10];

        // left
        {
            // cycles_counter::reset();
            // cycles_counter::start();

            HAL_GPIO_WritePin(ENC2_Cs_GPIO_Port, ENC2_Cs_Pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            auto status = HAL_SPI_Receive(&hspi2, buffer1, 10, 10);
            if (status != HAL_OK)
                logging::printf_blocking(100, "status was = %d\n", static_cast<int>(status));
            HAL_GPIO_WritePin(ENC2_Cs_GPIO_Port, ENC2_Cs_Pin, GPIO_PIN_SET);

            // cycles_counter::stop();
            // logging::printf_blocking(100, "reading took %d us\n", cycles_counter::get_us());
        }
        AS5045Reading left = AS5045Reading::from_buffer(buffer1);

        // right
        {
            HAL_GPIO_WritePin(ENC1_Cs_GPIO_Port, ENC1_Cs_Pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            auto status = HAL_SPI_Receive(&hspi2, buffer2, 10, 10);
            if (status != HAL_OK)
                logging::printf_blocking(100, "status was = %d\n", static_cast<int>(status));
            HAL_GPIO_WritePin(ENC1_Cs_GPIO_Port, ENC1_Cs_Pin, GPIO_PIN_SET);
        }
        AS5045Reading right = AS5045Reading::from_buffer(buffer2);

        const char *fmt1 = "LEFT: ang=%6d fs=%d pok=%d ok=%d   RIGHT: ang=%6d fs=%d pok=%d ok=%d\n";
        const char *fmt2 = "LEFT: %6d %d %d %d %d %d %d           RIGHT: %6d %d %d %d %d %d %d\n";
        const char *fmt3 = "LEFT: %06x                       RIGHT: %06x\n";

        logging::printf_blocking(200, fmt1,
                left.angle, static_cast<int>(left.field_status()), left.is_pairty_ok(), left.is_ok(),
                right.angle, static_cast<int>(right.field_status()), right.is_pairty_ok(), right.is_ok()
                );

        logging::printf_blocking(200, fmt2,
                left.angle, left.OCF, left.COF, left.LIN, left.magINC, left.magDEC, left.evenPAR,
                right.angle, right.OCF, right.COF, right.LIN, right.magINC, right.magDEC, right.evenPAR
                );
        logging::printf_blocking(200, fmt3,
                left._data,
                right._data
                );

        logging::printf_blocking(200, "buffer1 = [ %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x ]\n",
                buffer1[0], buffer1[1], buffer1[2], buffer1[3], buffer1[4],
                buffer1[5], buffer1[6], buffer1[7], buffer1[8], buffer1[9]);
        logging::printf_blocking(200, "buffer2 = [ %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x ]\n",
                buffer2[0], buffer2[1], buffer2[2], buffer2[3], buffer2[4],
                buffer2[5], buffer2[6], buffer2[7], buffer2[8], buffer2[9]);


        char text[20] = {0};
        uint8_t *buf = buffer2;
        for (int i = 0; i < 18; i++) {
            if (buf[i / 8] & (i % 8))
                text[i] = '1';
            else
                text[i] = '0';
        }
        logging::printf_blocking(200, "buf1 = %s\n", text);

        logging::printf_blocking(10, "\n");

        HAL_Delay(100);
    }











    /*** Prepare FreeRTOS tasks ***********************************************/

    // Most tasks are implemented as singletons with lazy-evaluation, i.e.
    // the object is constructed on first call to get(), so we need to
    // create these tasks here, before starting the scheduler.

    bool all_created = true;
    // all_created &= xTaskCreate(dummy_task, "Dummy 1",
    //         configMINIMAL_STACK_SIZE + 64, (void *) 1, 2, nullptr) == pdPASS;
    // all_created &= xTaskCreate(dummy_task, "Dummy 2",
    //         configMINIMAL_STACK_SIZE + 64, (void *) 2, 2, nullptr) == pdPASS;
    all_created &= xTaskCreate(stats_task, "Stats",
            configMINIMAL_STACK_SIZE *  2, nullptr, 1, nullptr) == pdPASS;
    configASSERT(all_created);

    /*** Print debug memory debug information *********************************/

    // this allows to optimize heap size that we assigned in FreeRTOSConfig.h
    // (more heap can be needed if anything is created dynamically later)
    size_t heap_size_remaining = xPortGetFreeHeapSize();
    logging::printf_blocking(60, "Remaining heap size = %u KB (%u B)\n",
            heap_size_remaining / (1 << 10), heap_size_remaining);

    /*** Start FreeRTOS scheduler *********************************************/

    logging::printf_blocking(100, "Starting scheduler...\n");

    // start RTOS event loop
    // IMPORTANT NOTE! this resets stack pointer, so all variables declared in
    // this scope will be overwritten - if needed, declare them globally or
    // allocate dynamically
    vTaskStartScheduler();

    // execution should never reach here, if it did, then something went wrong
    configASSERT(0);
}
