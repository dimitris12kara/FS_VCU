/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
extern ACUData acu;
extern IsabellenData ivt;
extern DataLoggerData dataLogger;
extern InverterData inverter;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef  sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = ACU_INFO_1_RX_CAN_ID << 5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = ACU_INFO_2_RX_CAN_ID << 5;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) Error_Handler();

  sFilterConfig.FilterBank = 1;
  sFilterConfig.FilterIdHigh = INTERFACE_IS_CONNECTED_RX_CAN_ID << 5;
  sFilterConfig.FilterMaskIdHigh = ACU_IS_CONNECTED_RX_CAN_ID << 5;
  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) Error_Handler();

  sFilterConfig.FilterBank = 2;
  sFilterConfig.FilterIdHigh = IVT_CURRENT_CAN_ID << 5;
  sFilterConfig.FilterMaskIdHigh = IVT_CURRENT_COUNTER_CAN_ID << 5;
  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) Error_Handler();

  sFilterConfig.FilterBank = 3;
  sFilterConfig.FilterIdHigh = IVT_WATTAGE_CAN_ID << 5;
  sFilterConfig.FilterMaskIdHigh = IVT_WATTAGE_COUNTER_CAN_ID << 5;
  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) Error_Handler();

  sFilterConfig.FilterBank = 4;
  sFilterConfig.FilterIdHigh = IVT_VOLTAGE_1_CAN_ID << 5;
  sFilterConfig.FilterMaskIdHigh =  INVERTER_RX_CAN_ID << 5;
  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) Error_Handler();

  sFilterConfig.FilterBank = 5;
  sFilterConfig.FilterIdHigh = 0x444 << 5;
  sFilterConfig.FilterMaskIdHigh =  INVERTER_RX_CAN_ID << 5;
  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) Error_Handler();


  if(HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler ();

//  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)  Error_Handler();

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 10;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterTypeDef  sFilterConfig;
  sFilterConfig.FilterBank = 14;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = THERMISTORS_RX_CAN_ID << 5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = DATALOGGER_INFO_RX_CAN_ID << 5;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) Error_Handler();

  sFilterConfig.FilterBank = 15;
  sFilterConfig.FilterIdHigh = AERO_PRESSURE_BME_1_RX_CAN_ID << 5;
  sFilterConfig.FilterMaskIdHigh = AERO_PRESSURE_MS55_1_RX_CAN_ID << 5;
  if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) Error_Handler();

  sFilterConfig.FilterBank = 16;
  sFilterConfig.FilterIdHigh = REAR_WHEEL_RPM_RX_CAN_ID << 5;
  sFilterConfig.FilterMaskIdHigh = REAR_WHEEL_RPM_RX_CAN_ID << 5;
  if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) Error_Handler();

//
//  sFilterConfig.FilterBank = 17;
//  sFilterConfig.FilterIdHigh = 0x176<<5;
//  sFilterConfig.FilterMaskIdHigh = 0x105<<5;
//  if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) Error_Handler();



  if (HAL_CAN_Start(&hcan2) != HAL_OK) Error_Handler ();

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t canflag = 0;
uint8_t canVar = 0;
int16_t tempCurrent = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CANData _canData;
  if (HAL_CAN_GetRxMessage(&criticalCan, CAN_RX_FIFO0, &_canData.RxHeader, _canData.RxData) != HAL_OK) Error_Handler();
  canflag++;
  if(_canData.RxHeader.StdId == IVT_CURRENT_CAN_ID){
	  ivt.timeoutFlag = 0;
	  ivt.current = (float)((_canData.RxData[2] << 24) + (_canData.RxData[3] << 16) + (_canData.RxData[4] << 8) + _canData.RxData[5]) / 1000.0;
  }
  else if(_canData.RxHeader.StdId == 0x444){
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
	  canVar++;
  }
  else if(_canData.RxHeader.StdId == IVT_VOLTAGE_1_CAN_ID) ivt.voltage1 = (float)((_canData.RxData[2] << 24) + (_canData.RxData[3] << 16) + (_canData.RxData[4] << 8) + _canData.RxData[5]) / 1000.0;
//  else if(_canData.RxHeader.StdId == IVT_VOLTAGE_2_CAN_ID) ivt.voltage2 = (_canData.RxData[2] << 24) + (_canData.RxData[3] << 16) + (_canData.RxData[4] << 8) + _canData.RxData[5];
  else if(_canData.RxHeader.StdId == ACU_INFO_1_RX_CAN_ID){
	  acu.SCSLEDsTimeOutTimer.triggered = 0;
	  acu.SCSLEDsTimeOutTimer.timer->Instance->CNT = 0;
	  acu.SCSLEDsTemporaryCheckSum = 0;
	  acu.AMSHasLatchedError = _canData.RxData[0] & 0x01;
	  acu.IMDHasError = (_canData.RxData[0] >> 1) & 0x01;
	  acu.AIRPlusIsArmed = (_canData.RxData[0] >> 2) & 0x01;
	  acu.AIRMinusIsArmed = (_canData.RxData[0] >> 3) & 0x01;
	  acu.PREIsArmed = (_canData.RxData[0] >> 4) & 0x01;
	  acu.highestCellTemp = _canData.RxData[1];
	  acu.maxVoltageCell.val = getVoltage(_canData.RxData[2] << 8 | _canData.RxData[3]);
	  acu.minVoltageCell.val = getVoltage(_canData.RxData[4] << 8 | _canData.RxData[5]);
	  for(int i = 0; i < 7; i++) acu.SCSLEDsTemporaryCheckSum += _canData.RxData[i];
	  if(acu.SCSLEDsTemporaryCheckSum != _canData.RxData[7]) acu.SCSLEDsCheckSumCurrentErrors++;
	  else acu.SCSLEDsCheckSumCurrentErrors = 0;
  }
  else if(_canData.RxHeader.StdId == ACU_INFO_2_RX_CAN_ID){
	  acu.AMSHasError = _canData.RxData[0] & 0b111;
	  acu.AIRsAreStuck = (_canData.RxData[0] >> 3) & 0b11;
	  acu.TSOver60Volt = (_canData.RxData[0] >> 5) & 0b1;
	  acu.maxCellTemp.id = _canData.RxData[1];
	  acu.maxVoltageCell.id = _canData.RxData[2];
	  acu.minVoltageCell.id = _canData.RxData[3];
	  acu.IMDResistance = 90 * 1200 / (map(_canData.RxData[4], 0, 255, 0, 100) - 5) - 1200;
	  acu.humidity = _canData.RxData[5];
	  acu.PCBTemperature = _canData.RxData[6];
	  acu.vicorTemperature = _canData.RxData[7];

  }
  else if(_canData.RxHeader.StdId == INVERTER_RX_CAN_ID){
	  inverter.timeOutTimer.triggered = TIMER_RESET;
	  inverter.timeOutTimer.timer->Instance->CNT = 0;
	  inverter.isConnected = 1;
	  switch(_canData.RxData[0]){
	  case INVERTER_SPEED_COMMAND:
		  int16_t tempRPM = (_canData.RxData[2] * 256) | _canData.RxData[1];
		  inverter.RPMSpeed = (float)(tempRPM *  MAX_RPM / 32767.0);
		  if(inverter.RPMSpeed < 0) inverter.RPMSpeed = 0 - inverter.RPMSpeed;
//		  dataLogger.vehicleSpeed = inverter.RPMSpeed / 3.98 * 2 * PI * WHEEL_RADIUS;
	  break;

	  case INVERTER_MOTOR_TEMP_COMMAND:
		  inverter.motorTemp = map((_canData.RxData[2] << 8) | _canData.RxData[1], 9939, 14890, 10, 100);
	  break;

	  case INVERTER_IGBT_TEMP_COMMAND:
		  inverter.igbtTemp = map((_canData.RxData[2] << 8) | _canData.RxData[1], 18017, 27114, 10, 100);
	  break;
	  case INVERTER_ACTUAL_CURRENT_COMMAND:
		  int16_t tempCurrent = ((_canData.RxData[2] << 8) | _canData.RxData[1]);
		  inverter.actualCurrent = 0.2 * (inverter.currentDevice * tempCurrent) / inverter.current200pc;
	  break;
	  case INVERTER_ACTUAL_Q_CURRENT_COMMAND:
		  int16_t tempCurrentQ = ((_canData.RxData[2] << 8) | _canData.RxData[1]);
		  inverter.actualCurrentQ = 0.2 * (inverter.currentDevice * tempCurrentQ) / inverter.current200pc;
	  break;

//	  case INVERTER_CURRENT_200_PC_COMMAND:
//		  inverter.current200pc = ((_canData.RxData[2] << 8) | _canData.RxData[1]);
//	  break;
//	  case 0xc6:
//		  tempCurrent = ((_canData.RxData[2] << 8) | _canData.RxData[1]);
//	  break;
	  default:
	  break;
	  }
  }
  else if(_canData.RxHeader.StdId == IVT_WATTAGE_CAN_ID) ivt.wattage = (_canData.RxData[2] << 24) + (_canData.RxData[3] << 16) + (_canData.RxData[4] << 8) + _canData.RxData[5];
  else if(_canData.RxHeader.StdId == IVT_CURRENT_COUNTER_CAN_ID) ivt.currentCounter = (_canData.RxData[2] << 24) + (_canData.RxData[3] << 16) + (_canData.RxData[4] << 8) + _canData.RxData[5];
  else if(_canData.RxHeader.StdId == IVT_WATTAGE_COUNTER_CAN_ID) ivt.wattageCounter = (_canData.RxData[2] << 24) + (_canData.RxData[3] << 16) + (_canData.RxData[4] << 8) + _canData.RxData[5];
  else if(_canData.RxHeader.StdId == INTERFACE_IS_CONNECTED_RX_CAN_ID) acu.userTimeOutFlag = 0;
  else if(_canData.RxHeader.StdId == ACU_IS_CONNECTED_RX_CAN_ID) acu.isConnected = 1;
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CANData _canData;
  if (HAL_CAN_GetRxMessage(&sensorsCan, CAN_RX_FIFO1, &_canData.RxHeader, _canData.RxData) != HAL_OK) Error_Handler();

  if(_canData.RxHeader.StdId == THERMISTORS_RX_CAN_ID) {
	  dataLogger.controlBoxTimeOutFlag = 0;
	  for(int i = 0; i < 6; i++) dataLogger.coolingThermistors[i] = _canData.RxData[i];
  }
  else if(_canData.RxHeader.StdId == DATALOGGER_INFO_RX_CAN_ID){
	  dataLogger.mainTimeOutFlag = 0;
	  dataLogger.frontLeftWheelRPM = ((int16_t)(_canData.RxData[0] << 8 | _canData.RxData[1]));
	  dataLogger.frontRightWheelRPM = ((int16_t)(_canData.RxData[2] << 8 | _canData.RxData[3]));
	  dataLogger.hours = _canData.RxData[4];
	  dataLogger.minutes = _canData.RxData[5];

  }
  else if((_canData.RxHeader.StdId == REAR_WHEEL_RPM_RX_CAN_ID)){// 20 ms
//	    sensorHall[0] = sensorsRxData[0];
//	    sensorHall[1] = sensorsRxData[2];
	  dataLogger.rearLeftWheelRPM = ((int16_t)(_canData.RxData[0] << 8 | _canData.RxData[1]));
	  dataLogger.rearRightWheelRPM = ((int16_t)(_canData.RxData[2] << 8 | _canData.RxData[3]));
  }
  else if(_canData.RxHeader.StdId == AERO_PRESSURE_BME_1_RX_CAN_ID) {
	  dataLogger.aeroTimeOutFlag = 0;
	  dataLogger.aeroDataLoggerMode = 1;
  }
  else if(_canData.RxHeader.StdId == AERO_PRESSURE_MS55_1_RX_CAN_ID) {
	  dataLogger.aeroTimeOutFlag = 0;
	  dataLogger.aeroDataLoggerMode = 2;
  }





  //EDO tha pairno dedomena apo dataloggers gia othonh

}

/* USER CODE END 1 */
