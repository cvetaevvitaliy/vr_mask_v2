/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED_STATUS_Pin GPIO_PIN_13
#define LED_STATUS_GPIO_Port GPIOC
#define LED_STATUS_2_Pin GPIO_PIN_14
#define LED_STATUS_2_GPIO_Port GPIOC
#define POWER_KEY_Pin GPIO_PIN_0
#define POWER_KEY_GPIO_Port GPIOA
#define BLUETOOTH_UP_Pin GPIO_PIN_1
#define BLUETOOTH_UP_GPIO_Port GPIOA
#define ADC_Pin GPIO_PIN_4
#define ADC_GPIO_Port GPIOA
#define CHRG_STATUS_Pin GPIO_PIN_5
#define CHRG_STATUS_GPIO_Port GPIOA
#define VAPE_9_Pin GPIO_PIN_6
#define VAPE_9_GPIO_Port GPIOA
#define TIM3_CH2_FAN_L_Pin GPIO_PIN_7
#define TIM3_CH2_FAN_L_GPIO_Port GPIOA
#define TIM3_CH3_FAN_R_Pin GPIO_PIN_0
#define TIM3_CH3_FAN_R_GPIO_Port GPIOB
#define TIM3_CH4_Vibro_Pin GPIO_PIN_1
#define TIM3_CH4_Vibro_GPIO_Port GPIOB
#define VAPE_8_Pin GPIO_PIN_10
#define VAPE_8_GPIO_Port GPIOB
#define VAPE_7_Pin GPIO_PIN_11
#define VAPE_7_GPIO_Port GPIOB
#define Fan_smell_Pin GPIO_PIN_15
#define Fan_smell_GPIO_Port GPIOB
#define VAPE_6_Pin GPIO_PIN_8
#define VAPE_6_GPIO_Port GPIOA
#define VAPE_5_Pin GPIO_PIN_9
#define VAPE_5_GPIO_Port GPIOA
#define VAPE_4_Pin GPIO_PIN_10
#define VAPE_4_GPIO_Port GPIOA
#define VAPE_3_Pin GPIO_PIN_11
#define VAPE_3_GPIO_Port GPIOA
#define VAPE_2_Pin GPIO_PIN_15
#define VAPE_2_GPIO_Port GPIOA
#define VAPE_1_Pin GPIO_PIN_3
#define VAPE_1_GPIO_Port GPIOB
#define TIM4_CH1_WATER_L_Pin GPIO_PIN_6
#define TIM4_CH1_WATER_L_GPIO_Port GPIOB
#define TIM4_CH2_WATER_R_Pin GPIO_PIN_7
#define TIM4_CH2_WATER_R_GPIO_Port GPIOB
#define TIM4_CH3_HEALT_L_Pin GPIO_PIN_8
#define TIM4_CH3_HEALT_L_GPIO_Port GPIOB
#define TIM4_CH4_HEALT_R_Pin GPIO_PIN_9
#define TIM4_CH4_HEALT_R_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

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

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
