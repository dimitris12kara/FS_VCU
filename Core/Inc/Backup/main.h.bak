/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "utils.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define BRAKELIGHT_THRESHOLD ((uint16_t) 1000)
#define R2D_BRAKE_THRESHOLD ((uint16_t) 1000)
#define SCS_ACCEPTED_ERRORS ((uint16_t) 5)
#define MAPPED_THRESHOLD_OFFSET 10
#define ERROR_CHECKED_OFFSET 500
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define R2D_Output_Pin GPIO_PIN_1
#define R2D_Output_GPIO_Port GPIOC
#define R2D_Output_LED_Pin GPIO_PIN_2
#define R2D_Output_LED_GPIO_Port GPIOC
#define Programmable_LED_2_Pin GPIO_PIN_4
#define Programmable_LED_2_GPIO_Port GPIOA
#define Programmable_LED_1_Pin GPIO_PIN_6
#define Programmable_LED_1_GPIO_Port GPIOA
#define CAN2_LED_Pin GPIO_PIN_4
#define CAN2_LED_GPIO_Port GPIOC
#define CAN1_LED_Pin GPIO_PIN_5
#define CAN1_LED_GPIO_Port GPIOC
#define MENU_2_Button_Pin GPIO_PIN_6
#define MENU_2_Button_GPIO_Port GPIOC
#define MENU_2_Button_EXTI_IRQn EXTI9_5_IRQn
#define BrakeControl_Pin GPIO_PIN_7
#define BrakeControl_GPIO_Port GPIOC
#define MENU_1_Button_Pin GPIO_PIN_8
#define MENU_1_Button_GPIO_Port GPIOA
#define MENU_1_Button_EXTI_IRQn EXTI9_5_IRQn
#define DISPLAY_TX_Pin GPIO_PIN_9
#define DISPLAY_TX_GPIO_Port GPIOA
#define DISPLAY_RX_Pin GPIO_PIN_10
#define DISPLAY_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define R2D_Button_Pin GPIO_PIN_10
#define R2D_Button_GPIO_Port GPIOC
#define R2D_Button_EXTI_IRQn EXTI15_10_IRQn
#define IMD_LED_Pin GPIO_PIN_11
#define IMD_LED_GPIO_Port GPIOC
#define TSact_Button_Pin GPIO_PIN_12
#define TSact_Button_GPIO_Port GPIOC
#define TSact_Button_EXTI_IRQn EXTI15_10_IRQn
#define AMS_LED_Pin GPIO_PIN_2
#define AMS_LED_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define BuzzerControl_Pin GPIO_PIN_5
#define BuzzerControl_GPIO_Port GPIOB
#define NOT_WC_Pin GPIO_PIN_6
#define NOT_WC_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_7
#define EEPROM_SDA_GPIO_Port GPIOB
#define EEPROM_SCL_Pin GPIO_PIN_8
#define EEPROM_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define displayUart huart1
#define criticalCan hcan1
#define sensorsCan hcan2
#define pedalBoxADC hadc1
#define START_STATE_TIMER htim7
#define SCS_TIMEOUT_TIMER htim10
#define INVERTER_TIMEOUT_TIMER htim11
#define ADC_SAMPLE_RATE_TIMER htim8
#define eepromI2C hi2c1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
