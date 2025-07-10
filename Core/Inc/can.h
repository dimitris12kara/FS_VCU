/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
//CAN IDs

//CAN ID CRITICAL CAN

//Inverter IDs

#define INVERTER_TX_CAN_ID 0x201

#define INVERTER_RX_CAN_ID 0x181

//IVT IDs

#define IVT_CURRENT_CAN_ID 0x521

#define IVT_VOLTAGE_1_CAN_ID 0x522

#define IVT_VOLTAGE_2_CAN_ID 0x523

#define IVT_TEMPERATURE_CAN_ID 0x525

#define IVT_WATTAGE_CAN_ID 0x526

#define IVT_CURRENT_COUNTER_CAN_ID 0x527

#define IVT_WATTAGE_COUNTER_CAN_ID 0x528

//ACU IDS

#define INTERFACE_IS_CONNECTED_RX_CAN_ID 0x150

#define ACU_START_PRECHARGE_TX_CAN_ID 0x100

#define ACU_INFO_1_RX_CAN_ID 0x105

#define ACU_INFO_2_RX_CAN_ID 0x106

#define ACU_IS_CONNECTED_TX_CAN_ID 0x175

#define ACU_IS_CONNECTED_RX_CAN_ID 0x176

//CAN ID SENSOR CAN

#define DATALOGGER_ACTIVATION_TX_CAN_ID 0x103

#define PEDALS_TX_CAN_ID 0x104

#define THERMISTORS_RX_CAN_ID 0x453

#define DATALOGGER_INFO_RX_CAN_ID 0x444

#define AERO_PRESSURE_BME_1_RX_CAN_ID 0x603

#define AERO_PRESSURE_MS55_1_RX_CAN_ID 0x606

#define REAR_WHEEL_RPM_RX_CAN_ID 0x454

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

