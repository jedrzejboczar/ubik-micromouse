#include "driver.h"

#include <algorithm>
#include <cassert>

#include "stm32f1xx.h"

/*
 * Timer configured for PWM output.
 * We'll just assume that htim.Init holds real values.
 * TIM3 is clocked from APB1.
 */
extern TIM_HandleTypeDef htim3;


namespace movement {
namespace driver {


static TIM_HandleTypeDef &motors_timer = htim3;

static constexpr uint32_t left_tim_channel = TIM_CHANNEL_1;
static constexpr uint32_t right_tim_channel = TIM_CHANNEL_2;

static bool motors_enabled = false;
static MotorDirection left_direction = STOPPED;
static MotorDirection right_direction = STOPPED;


static void MOT1_Dir1(bool state) {
    HAL_GPIO_WritePin(MOT1_Dir1_GPIO_Port, MOT1_Dir1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void MOT1_Dir2(bool state) {
    HAL_GPIO_WritePin(MOT1_Dir2_GPIO_Port, MOT1_Dir2_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void MOT2_Dir1(bool state) {
    HAL_GPIO_WritePin(MOT2_Dir1_GPIO_Port, MOT2_Dir1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void MOT2_Dir2(bool state) {
    HAL_GPIO_WritePin(MOT2_Dir2_GPIO_Port, MOT2_Dir2_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


unsigned int max_pulse() {
    return motors_timer.Init.Period;
}



void initialise() {
    HAL_StatusTypeDef result;
	result = HAL_TIM_PWM_Start(&motors_timer, left_tim_channel);
    assert(result == HAL_OK);
	result = HAL_TIM_PWM_Start(&motors_timer, right_tim_channel);
    assert(result == HAL_OK);
}

void set_enabled(bool enabled) {
    if (enabled) {
        // TODO: I2C1 cannot be enabled while right motor is enabled
        __HAL_RCC_I2C1_CLK_DISABLE();

        motors_enabled = true;
        set_direction(left_direction, right_direction);
    } else {
        motors_enabled = false;
		MOT1_Dir1(false);
		MOT1_Dir2(false);
		MOT2_Dir1(false);
		MOT2_Dir2(false);
    }
}

static_assert(left_tim_channel == TIM_CHANNEL_1, "Adjust set_pulse if channel changes.");
static_assert(right_tim_channel == TIM_CHANNEL_2, "Adjust set_pulse if channel changes.");

void set_pulse(unsigned int left, unsigned int right) {
    motors_timer.Instance->CCR1 = std::min(left, max_pulse());
    motors_timer.Instance->CCR2 = std::min(right, max_pulse());
}

void set_direction(MotorDirection left, MotorDirection right) {
    left_direction = left;
    right_direction = right;
    if (motors_enabled) {
        switch (left) {
            case FORWARDS:
                MOT1_Dir1(false);
                MOT1_Dir2(true);
                break;
            case BACKWARDS:
                MOT1_Dir1(true);
                MOT1_Dir2(false);
                break;
            case STOPPED:
                MOT1_Dir1(true);
                MOT1_Dir2(true);
                break;
            default: break;
        }
        switch (right) {
            case FORWARDS:
                MOT2_Dir1(false);
                MOT2_Dir2(true);
                break;
            case BACKWARDS:
                MOT2_Dir1(true);
                MOT2_Dir2(false);
                break;
            case STOPPED:
                MOT2_Dir1(true);
                MOT2_Dir2(true);
                break;
            default: break;
        }
    }
}


} // namespace driver
} // namespace movement

