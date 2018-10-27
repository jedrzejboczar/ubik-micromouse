/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LS_Power_Pin GPIO_PIN_13
#define LS_Power_GPIO_Port GPIOC
#define GPIO_in_unused1_Pin GPIO_PIN_14
#define GPIO_in_unused1_GPIO_Port GPIOC
#define GPIO_in_unused2_Pin GPIO_PIN_15
#define GPIO_in_unused2_GPIO_Port GPIOC
#define DS1_Read_Pin GPIO_PIN_0
#define DS1_Read_GPIO_Port GPIOA
#define DS2_Read_Pin GPIO_PIN_1
#define DS2_Read_GPIO_Port GPIOA
#define DS3_Read_Pin GPIO_PIN_2
#define DS3_Read_GPIO_Port GPIOA
#define DS4_Read_Pin GPIO_PIN_3
#define DS4_Read_GPIO_Port GPIOA
#define DS5_Read_Pin GPIO_PIN_4
#define DS5_Read_GPIO_Port GPIOA
#define DS6_Read_Pin GPIO_PIN_5
#define DS6_Read_GPIO_Port GPIOA
#define MEAS_Curr_Pin GPIO_PIN_6
#define MEAS_Curr_GPIO_Port GPIOA
#define MEAS_Volt_Pin GPIO_PIN_7
#define MEAS_Volt_GPIO_Port GPIOA
#define LS_Com_Read_Pin GPIO_PIN_0
#define LS_Com_Read_GPIO_Port GPIOB
#define LS_Adr0_Pin GPIO_PIN_1
#define LS_Adr0_GPIO_Port GPIOB
#define LS_Adr1_Pin GPIO_PIN_2
#define LS_Adr1_GPIO_Port GPIOB
#define LS_Adr2_Pin GPIO_PIN_10
#define LS_Adr2_GPIO_Port GPIOB
#define GPIO_Exp_Cs_Pin GPIO_PIN_11
#define GPIO_Exp_Cs_GPIO_Port GPIOB
#define ENC1_Cs_Pin GPIO_PIN_12
#define ENC1_Cs_GPIO_Port GPIOB
#define SPI_Clk_Pin GPIO_PIN_13
#define SPI_Clk_GPIO_Port GPIOB
#define SPI_Miso_Pin GPIO_PIN_14
#define SPI_Miso_GPIO_Port GPIOB
#define SPI_Mosi_Pin GPIO_PIN_15
#define SPI_Mosi_GPIO_Port GPIOB
#define ENC2_Cs_Pin GPIO_PIN_8
#define ENC2_Cs_GPIO_Port GPIOA
#define BT_Rx_Pin GPIO_PIN_9
#define BT_Rx_GPIO_Port GPIOA
#define BT_Tx_Pin GPIO_PIN_10
#define BT_Tx_GPIO_Port GPIOA
#define BT_Key_Pin GPIO_PIN_11
#define BT_Key_GPIO_Port GPIOA
#define SW_Start_Pin GPIO_PIN_12
#define SW_Start_GPIO_Port GPIOA
#define SWD_IO_Pin GPIO_PIN_13
#define SWD_IO_GPIO_Port GPIOA
#define SDW_CLK_Pin GPIO_PIN_14
#define SDW_CLK_GPIO_Port GPIOA
#define MOT1_Dir1_Pin GPIO_PIN_15
#define MOT1_Dir1_GPIO_Port GPIOA
#define MOT1_Dir2_Pin GPIO_PIN_3
#define MOT1_Dir2_GPIO_Port GPIOB
#define MOT1_Pwm_Pin GPIO_PIN_4
#define MOT1_Pwm_GPIO_Port GPIOB
#define MOT2_Pwm_Pin GPIO_PIN_5
#define MOT2_Pwm_GPIO_Port GPIOB
#define MOT2_Dir2_Pin GPIO_PIN_6
#define MOT2_Dir2_GPIO_Port GPIOB
#define MOT2_Dir1_Pin GPIO_PIN_7
#define MOT2_Dir1_GPIO_Port GPIOB
#define IMU_Clk_Pin GPIO_PIN_8
#define IMU_Clk_GPIO_Port GPIOB
#define IMU_Data_Pin GPIO_PIN_9
#define IMU_Data_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
 #define USE_FULL_ASSERT    1U 

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
