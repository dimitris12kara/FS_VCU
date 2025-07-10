/*
 * utils.c
 *
 *  Created on: Dec 27, 2023
 *      Author: kara
 */
#include "main.h"

/**
  * @brief  Function that reads data from eeprom when LVMS is powered (only on start up).
  * @param  argument: I2C_HandleTypeDef, VCUData structure.
  * @retval
  */

HAL_StatusTypeDef retrieveVCUDataFromEepromOnStartUp(I2C_HandleTypeDef *_eepromI2C, VCUData *_vcu){
	uint8_t buf[15];

	readDataEeprom(_eepromI2C, EEPROM_STARTING_ID, buf, 14);
	_vcu->APPS1.mappedMin = (buf[0] << 8) | buf[1];
	_vcu->APPS2.mappedMin = (buf[2] << 8) | buf[3];
	_vcu->APPS1.mappedMax = (buf[4] << 8) | buf[5];
	_vcu->APPS2.mappedMax = (buf[6] << 8) | buf[7];
	_vcu->brakeSensor.mappedMin = (buf[8] << 8) | buf[9];
	_vcu->brakeSensor.mappedMax = (buf[10] << 8) | buf[11];
	_vcu->maxTorque = (buf[12] << 8) | buf[13];
//	_vcu->menuPage =  0b1111 & buf[14];
	return HAL_OK;
}
/**
  * @brief  Function that initializes VCUData structure.
  *   */


HAL_StatusTypeDef VCUInit(VCUData *_vcu, TIM_HandleTypeDef *_startStateTimer, TIM_HandleTypeDef *_ADCSampleRateTimer, uint16_t _brakeLightThreshold, uint16_t _brakeR2DThreshold){
	_vcu->state = OFF;
	_vcu->lastError = NONE_ERROR;
	_vcu->menuPage = MAIN_PAGE;
	_vcu->loadNewScreen = TO_ENTER;

	//Values need init for display//
	_vcu->brakeLightThreshold = _brakeLightThreshold;
	_vcu->brakeR2DThreshold = _brakeR2DThreshold;

	_vcu->brakeLight.pin = BrakeControl_Pin;
	_vcu->brakeLight.group = BrakeControl_GPIO_Port;

	_vcu->buzzer.pin = BuzzerControl_Pin;
	_vcu->buzzer.group = BuzzerControl_GPIO_Port;

	_vcu->R2DButton.pin = R2D_Button_Pin;
	_vcu->R2DButton.state = NOT_PRESSED;

	_vcu->tsActButton.pin = TSact_Button_Pin;
	_vcu->tsActButton.state = NOT_PRESSED;

	_vcu->utils1Button.pin = MENU_1_Button_Pin;
	_vcu->utils1Button.state = NOT_PRESSED;

	_vcu->utils2Button.pin = MENU_2_Button_Pin;
	_vcu->utils2Button.state = NOT_PRESSED;

	_vcu->offStateFirstLoopEnter = TO_ENTER;
	_vcu->prechargedStateFirstLoopEnter = TO_ENTER;
	_vcu->R2DStateFirstLoopEnter = TO_ENTER;

	_vcu->lowerDeadZoneThreshold = 200;
	_vcu->upperDeadZoneThreshold = 3900;

	_vcu->startStateTimer.triggered = 0;
	_vcu->startStateTimer.timer = _startStateTimer;

	_vcu->ADCSampleRateTimer.triggered = 0;
	_vcu->ADCSampleRateTimer.timer = _ADCSampleRateTimer;

	_vcu->rawSensorData[0] = 0;
	_vcu->rawSensorData[1] = 0;
	_vcu->rawSensorData[2] = 0;

	for(int i = 0; i < MOVING_AVERAGE; i++){
		_vcu->APPS1.sampleArray[i] = 0;
		_vcu->APPS2.sampleArray[i] = 0;
		_vcu->brakeSensor.sampleArray[i] = 0;

	}

	_vcu->APPS1.mappedMin = 10;
	_vcu->APPS1.mappedMax = 3000;
	_vcu->APPS1.sum = 0;
	_vcu->APPS1.filtered = 0;

	_vcu->APPS2.mappedMin = 10;
	_vcu->APPS2.mappedMax = 3000;
	_vcu->APPS2.sum = 0;
	_vcu->APPS2.filtered = 0;

	_vcu->brakeSensor.mappedMin = 0;
	_vcu->brakeSensor.mappedMax = ADC_MAX_VALUE - 1000;
	_vcu->brakeSensor.sum = 0;
	_vcu->brakeSensor.filtered = 0;
	_vcu->brakePressure = 0;
	_vcu->brakeSensorRange = 2500; //PSI

	_vcu->pedalSensorFinalValue = 0;
	_vcu->brakeSensorFinalValue = 0;

	_vcu->telemetryIsConnected = 0;
	_vcu->telemetryTimeoutFlag = 0;
	if(HAL_TIM_Base_Start_IT(_ADCSampleRateTimer) != HAL_OK) Error_Handler();

	return HAL_TIM_Base_Start_IT(_startStateTimer);
}
/**
  * @brief  Function that initializes ACUData structure.
  *   */

HAL_StatusTypeDef ACUInit(ACUData *_acu, TIM_HandleTypeDef *_SCStimeOutTimer, uint8_t _SCSLEDsCheckSumAcceptedErrors){


	_acu->vicorTemperature = 3;
	_acu->isConnected = 0;
	_acu->SCSLEDsAreConnected = 0;
	_acu->AIRMinusIsArmed = 0;
	_acu->AIRPlusIsArmed = 0;
	_acu->PREIsArmed = 0;
	_acu->highestCellTemp = 1;

	_acu->humidity = -2.4;
	_acu->PCBTemperature = 0;
	_acu->IMDResistance = 0;
	_acu->soc = 0;

	_acu->SCSLEDsCheckSumCurrentErrors = 0;

	_acu->SCSLEDsCheckSumAcceptedErrors = _SCSLEDsCheckSumAcceptedErrors;

	_acu->SCSLEDsTimeOutTimer.triggered = 0;
	_acu->SCSLEDsTimeOutTimer.timer = _SCStimeOutTimer;

	_acu->AMSHasError = 1;
	_acu->AMSHasLatchedError = 1;
	_acu->IMDHasError = 1;
	_acu->SCSLEDsAreConnected = 0;
	_acu->AIRsAreStuck = 0;
	_acu->SCSLEDsTemporaryCheckSum = 0;
	_acu->TSOver60Volt = 0;

	//Pins
	//Led for AMS error in dashboard.
	_acu->AMSLed.group = AMS_LED_GPIO_Port;
	_acu->AMSLed.pin = AMS_LED_Pin;

	//Led for IMD error in dashboard.

	_acu->IMDLed.group = IMD_LED_GPIO_Port;
	_acu->IMDLed.pin = IMD_LED_Pin;


	_acu->maxVoltageCell.val = 7.0;
	_acu->minVoltageCell.val = 1.1;
	_acu->maxCellTemp.val = 2;
	_acu->maxVoltageCell.id = 7.0;
	_acu->minVoltageCell.id = 1.1;
	_acu->maxCellTemp.id = 3;
	HAL_GPIO_WritePin(acu.IMDLed.group, acu.IMDLed.pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(acu.AMSLed.group, acu.AMSLed.pin, GPIO_PIN_SET);
	// Enable CallBack functions as late as possible to avoid hardfaults with RAM and not correct initialization of structures.

	  CANData canData;
	  canData.TxHeader.IDE = CAN_ID_STD;
	  canData.TxHeader.StdId = ACU_IS_CONNECTED_TX_CAN_ID;
	  canData.TxHeader.RTR = CAN_RTR_DATA;
	  canData.TxHeader.DLC = 1;
	  canData.TxData[0] = 1;
	  if (HAL_CAN_ActivateNotification(&criticalCan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)  Error_Handler();
//	  while(!acu.isConnected){
//		  if(HAL_CAN_AddTxMessage(&criticalCan, &canData.TxHeader, canData.TxData, &canData.TxMailbox) != HAL_OK) Error_Handler();
//		  osDelay(10);
//	  }
	  return HAL_TIM_Base_Start_IT(_SCStimeOutTimer);// Starts timer of SCS Timeout Timer.
}

/**
  * @brief  Function that initializes DataLogger structure.
  *   */
HAL_StatusTypeDef DataloggerInit(DataLoggerData *_datalogger){
	for(int i = 0; i < 6; i++) _datalogger->coolingThermistors[i] = 0;
	_datalogger->aeroIsConnected = 0;
	_datalogger->aeroTimeOutFlag = 0;
	_datalogger->controlBoxIsConnected = 0;
	_datalogger->controlBoxTimeOutFlag = 0;
	_datalogger->mainIsConnected = 0;
	_datalogger->mainTimeOutFlag = 0;
	_datalogger->wantedDutyCycle = 0;
	_datalogger->vehicleSpeed = 0;
	_datalogger->maxAcc = 0.1;
	_datalogger->SteeringAngle = 0;
	_datalogger->wheelRPM = 0;
	_datalogger->motorRPM = 0;
	_datalogger->arePoweredOn = 0;
	_datalogger->aeroDataLoggerMode = 0;
	_datalogger->rearLeftWheelRPM = 0;
	_datalogger->rearRightWheelRPM = 0;
	_datalogger->frontLeftWheelRPM = 0;
	_datalogger->frontRightWheelRPM = 0;
	_datalogger->minutes = 0;
	_datalogger->hours = 0;

	return HAL_OK;
}

HAL_StatusTypeDef InverterInit(InverterData *_inverter, CAN_HandleTypeDef *_hcan, TIM_HandleTypeDef *_timeOutTimer){
	_inverter->DCBus = 0;
	_inverter->igbtTemp = 0;
	_inverter->motorTemp = 0;
	_inverter->RPMSpeed = 0;
	_inverter->run.group = R2D_Output_GPIO_Port;
	_inverter->run.pin = R2D_Output_Pin;

	_inverter->current1AC = 0;
	_inverter->current2AC = 0;
	_inverter->current3AC = 0;
	_inverter->actualCurrent = 0;
	_inverter->actualCurrentQ = 0;
	_inverter->current200pc = 1070;
	_inverter->currentDevice = 2000;
	_inverter->timeOutTimer.triggered  = TIMER_RESET;
	_inverter->timeOutTimer.timer = _timeOutTimer;

	// Ask values to be transmitted from motor controller.
	askInverterValue(_hcan, INVERTER_SPEED_COMMAND, 100);
	osDelay(INVERTER_DELAY);
	askInverterValue(_hcan, INVERTER_MOTOR_TEMP_COMMAND, 200);
	osDelay(INVERTER_DELAY);
	askInverterValue(_hcan, INVERTER_IGBT_TEMP_COMMAND, 200);
	osDelay(INVERTER_DELAY);
	askInverterValue(_hcan, INVERTER_ACTUAL_CURRENT_COMMAND, 100);
//	osDelay(INVERTER_DELAY);
//	askInverterValue(_hcan, INVERTER_CURRENT_200_PC_COMMAND, 200);
	osDelay(INVERTER_DELAY);
	askInverterValue(_hcan, INVERTER_ACTUAL_Q_CURRENT_COMMAND, 100);
//	osDelay(INVERTER_DELAY);
//	askInverterValue(_hcan, 0xc6, 200);
//	askInverterValue(_hcan,INVERTER_CURRENT_1_AC_COMMAND,200);
//	osDelay(INVERTER_DELAY);
//	askInverterValue(_hcan,INVERTER_CURRENT_2_AC_COMMAND,200);
//	osDelay(INVERTER_DELAY);
//	askInverterValue(_hcan,INVERTER_CURRENT_3_AC_COMMAND,200);
//	osDelay(INVERTER_DELAY);
	deactivateInverter(_hcan, _inverter);// Disables the inverter through CANBus and Analog Signal (RUN).
	//DeactivateInverter!





	return HAL_TIM_Base_Start_IT(_timeOutTimer);
}

HAL_StatusTypeDef IVTInit(IsabellenData *_ivt){
	_ivt->isConnected = 0;
	_ivt->timeoutFlag = 0;
	_ivt->current = 0;
	_ivt->currentCounter = 0;
	_ivt->wattageCounter = 0;
	_ivt->voltage1 = 0;
	_ivt->voltage2 = 0;
	_ivt->voltage3 = 0;
	_ivt->wattage = 0;
	return HAL_OK;
}

int16_t getIntFromVoltage(float voltage){
	int16_t intVoltage = (voltage / 0.000150) - 10000;
	return intVoltage;
}

float getVoltage(int data)
{
    float voltage_float = ((data + 10000) * 0.000150);
    return voltage_float;
}

uint8_t displayEndCommand[3] = {0xFF,0xFF,0xFF};  // command end sequence

void error2Screen(UART_HandleTypeDef *_huart,char *error){
	char buf[50];
	int len = sprintf (buf, "page3.error.txt=\"%s\"", error);
	HAL_UART_Transmit(_huart, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(_huart, displayEndCommand, 3, 100);
	osDelay(5);

	len = sprintf(buf, "page 3");
	HAL_UART_Transmit(_huart, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(_huart, displayEndCommand, 3, 100);



}



void str2Screen(UART_HandleTypeDef *_huart, char *ID, char *string)
{
	char buf[30];
	int len = sprintf(buf, "%s.txt=\"%s\"", ID, string);
	HAL_UART_Transmit(_huart, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(_huart, displayEndCommand, 3, 100);
}

void bar2Screen(UART_HandleTypeDef *_huart, char *ID, float val){
	char buf[20];
	int len = sprintf(buf, "%s.val=%ld", ID, (uint32_t)(val));
	HAL_UART_Transmit(_huart, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(_huart, displayEndCommand, 3, 100);
}


void int2Screen(UART_HandleTypeDef *_huart, char *ID, float val){
	char buf[20];
	int len = sprintf(buf, "%s.val=%ld", ID, (uint32_t)(val * 10));
	osDelay(1);
	HAL_UART_Transmit(_huart, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(_huart, displayEndCommand, 3, 100);
}

void changeTextColor2Screen(UART_HandleTypeDef *_huart, char *ID, char *color){
	char buf[30];
	int len = sprintf(buf, "Display.%s.pco=%s", ID, color);
	HAL_UART_Transmit(_huart, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(_huart, displayEndCommand, 3, 100);

}
void changePage2Screen(UART_HandleTypeDef *_huart, char *page){
	char buf[10];
	int len = sprintf(buf, "page %s", page);
	HAL_UART_Transmit(_huart, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(_huart, displayEndCommand, 3, 100);


}
void devicesTimeOutTimers(VCUData *_vcu,ACUData *_acu, DataLoggerData *_datalogger, IsabellenData *_ivt){
	_vcu->telemetryTimeoutFlag++;
	_datalogger->aeroTimeOutFlag++;
	_datalogger->controlBoxTimeOutFlag++;
	_datalogger->mainTimeOutFlag++;
	_ivt->timeoutFlag++;
	_acu->userTimeOutFlag++;

	if(_vcu->telemetryTimeoutFlag > 20){
		_vcu->telemetryIsConnected = DEVICE_DISCONNECTED;
	}
	else _vcu->telemetryIsConnected = DEVICE_IS_CONNECTED;

	if(_datalogger->aeroTimeOutFlag > 20){
		_datalogger->aeroIsConnected = DEVICE_DISCONNECTED;
	}
	else _datalogger->aeroIsConnected = _datalogger->aeroDataLoggerMode;

	if(_datalogger->mainTimeOutFlag > 20){
		_datalogger->mainIsConnected = DEVICE_DISCONNECTED;
	}
	else _datalogger->mainIsConnected = DEVICE_IS_CONNECTED;

	if(_datalogger->controlBoxTimeOutFlag > 20){
		_datalogger->controlBoxIsConnected = DEVICE_DISCONNECTED;
	}
	else _datalogger->controlBoxIsConnected = DEVICE_IS_CONNECTED;

	if(_ivt->timeoutFlag > 20){
		_ivt->isConnected = DEVICE_DISCONNECTED;
	}
	else _ivt->isConnected = DEVICE_IS_CONNECTED;

	if(_acu->userTimeOutFlag > 50){
		_acu->userIsConnected = DEVICE_DISCONNECTED;
	}
	else _acu->userIsConnected = DEVICE_IS_CONNECTED;

	return;
}

//Map function is re ranging a variable linearly.
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  if(in_max == in_min) in_max ++;//To avoid hardfault errors (divided by zero)
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  if(in_max == in_min) in_max ++;//To avoid hardfault errors (divided by zero)
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void brakeSensor(VCUData *_vcu){
//	int16_t val = _vcu->brakeSensor.filtered;
	_vcu->brakePressure = 0.0689476 * (((fmap(_vcu->rawSensorData[2], 0, 4095, 0, 3.3) - 0.5) * _vcu->brakeSensorRange) / 4);
	int val = _vcu->rawSensorData[2];
	if((val < _vcu->brakeSensor.mappedMin - ERROR_CHECKED_OFFSET - 200|| val > _vcu->brakeSensor.mappedMax + ERROR_CHECKED_OFFSET + 1000)){
		_vcu->lastError = BRAKESENSOR_OUT_OF_RANGE;
		_vcu->brakeSensorFinalValue = -50;
		return;
	}
	val = map(val, _vcu->brakeSensor.mappedMin, _vcu->brakeSensor.mappedMax, 0, ADC_MAX_VALUE);
	if (val < _vcu->lowerDeadZoneThreshold * 3) _vcu->brakeSensorFinalValue = 0;
	if(val > _vcu->upperDeadZoneThreshold * 3) _vcu->brakeSensorFinalValue = ADC_MAX_VALUE;
	else _vcu->brakeSensorFinalValue = val;
		return;
}

void pedalSensor(VCUData *_vcu){
  int16_t val1 = _vcu->rawSensorData[1];//pairno analogika tis dio times ton apps
  int16_t val2 = _vcu->rawSensorData[0];//prepei na ginei ena mapping timon

  if(	  val1 < _vcu->APPS1.mappedMin - ERROR_CHECKED_OFFSET ||
		  val1 > _vcu->APPS1.mappedMax + ERROR_CHECKED_OFFSET ||
		  val2 < _vcu->APPS2.mappedMin - ERROR_CHECKED_OFFSET ||
		  val2 > _vcu->APPS2.mappedMax + ERROR_CHECKED_OFFSET)
  {
	  _vcu->pedalSensorFinalValue = -70;
	  _vcu->lastError = APPS_OUT_OF_RANGE_ERROR;
	  return;
  }

  val1 = map(val1, _vcu->APPS1.mappedMin, _vcu->APPS1.mappedMax, 0, ADC_MAX_VALUE);
  val2 = map(val2, _vcu->APPS2.mappedMin, _vcu->APPS2.mappedMax, 0, ADC_MAX_VALUE);

  if(abs(val1 - val2) > 0.1 * ADC_MAX_VALUE) {
    //T 11.8.8 elegxo an exo diafora timon gia 100ms
    for(int i = 0; i < 20; i++){
      val1 = _vcu->rawSensorData[1];//pairno analogika tis dio times ton apps
      val2 = _vcu->rawSensorData[0];//prepei na ginei ena mapping timon
      val1 = map(val1, _vcu->APPS1.mappedMin, _vcu->APPS1.mappedMax, 0, ADC_MAX_VALUE);
      val2 = map(val2, _vcu->APPS2.mappedMin, _vcu->APPS2.mappedMax, 0, ADC_MAX_VALUE);;
      if(abs(val1 - val2) < 0.2 * ADC_MAX_VALUE){
    	  if((val1 + val2) / 2 > _vcu->upperDeadZoneThreshold){
    		  _vcu->pedalSensorFinalValue = ADC_MAX_VALUE;
    		  return;
    	  }
    	  else if((val1 + val2) / 2 < _vcu->lowerDeadZoneThreshold) {
    		  _vcu->pedalSensorFinalValue = 0;
    		  return;
    	  }
    	  else {
    		  _vcu->pedalSensorFinalValue = (val1 + val2) / 2;
    		  return;
    	  }

      }
    osDelay(5);
    }
    //error2screen("Apps diff, return to state pre");
	_vcu->lastError = APPS_DIFF_ERROR;
	_vcu->pedalSensorFinalValue = -50;
    return;
  }

  if((val1 + val2) / 2 > _vcu->upperDeadZoneThreshold) _vcu->pedalSensorFinalValue = ADC_MAX_VALUE;


  else if((val1 + val2) / 2 < _vcu->lowerDeadZoneThreshold) _vcu->pedalSensorFinalValue = 0;

  else  _vcu->pedalSensorFinalValue = (val1 + val2) / 2;
  return;
}

//HAL_StatusTypeDef writeDataEeprom(I2C_HandleTypeDef *_eepromI2C, uint8_t page, uint8_t id, uint8_t *data, uint8_t length){
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//	uint16_t tmp = (page << 8) | id;
//	HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(_eepromI2C, EEPROM_ADRESS << 1, tmp, I2C_MEMADD_SIZE_16BIT, data, length, 1000);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
//	return ret;
//
//}

void writeDataEeprom(I2C_HandleTypeDef *_eepromI2C, uint32_t startingID, uint8_t *data, uint8_t length) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	osDelay(1);
	EEPROM_MultiByte_Write(_eepromI2C, (uint32_t)startingID, &data[0], length);
	osDelay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}
void readDataEeprom(I2C_HandleTypeDef *_eepromI2C, uint32_t startingID, uint8_t *data, uint8_t length) {
	EEPROM_MultiByte_Selective_Read(_eepromI2C, (uint32_t)startingID, &data[0], length);
}






void setTorque(CAN_HandleTypeDef *_hcan, int16_t trq){
	CANData _canData;

	trq = map(trq, 0, ADC_MAX_VALUE, 0, 32767);//66%->21845, //100%->32767
	_canData.TxHeader.IDE = CAN_ID_STD;
	_canData.TxHeader.StdId = INVERTER_TX_CAN_ID;//0x201
	_canData.TxHeader.RTR = CAN_RTR_DATA;
	_canData.TxHeader.DLC = 3;
	_canData.TxData[0] = INVERTER_SEND_TORQUE_COMMAND;
	_canData.TxData[1] = trq;
	_canData.TxData[2] = trq >> 8;
	if(HAL_CAN_AddTxMessage(_hcan, &_canData.TxHeader, _canData.TxData, &_canData.TxMailbox) != HAL_OK)	Error_Handler();
}

void askInverterValue(CAN_HandleTypeDef *_hcan,uint8_t regid, uint8_t dt){
	CANData _canData;

  //an thelo mia metavliti sinexomena ana dt xrono(0-255 ms)
	_canData.TxHeader.IDE = CAN_ID_STD;
	_canData.TxHeader.StdId = INVERTER_TX_CAN_ID;
	_canData.TxHeader.RTR = CAN_RTR_DATA;
	_canData.TxHeader.DLC = 3;
	_canData.TxData[0] = INVERTER_ASK_VALUE_COMMAND;
	_canData.TxData[1] = regid;
	_canData.TxData[2] = dt;
	if(HAL_CAN_AddTxMessage(_hcan, &_canData.TxHeader, _canData.TxData, &_canData.TxMailbox) != HAL_OK)	Error_Handler();
}

void deactivateInverter(CAN_HandleTypeDef *_hcan, InverterData *_inverter){
	CANData _canData;
	HAL_GPIO_WritePin(_inverter->run.group, _inverter->run.pin, GPIO_PIN_RESET);
	_canData.TxHeader.IDE = CAN_ID_STD;
	_canData.TxHeader.StdId = INVERTER_TX_CAN_ID;
	_canData.TxHeader.RTR = CAN_RTR_DATA;
	_canData.TxHeader.DLC = 3;
	_canData.TxData[0] = INVERTER_CONTROL_MODE_COMMAND;
	_canData.TxData[1] = INVERTER_DISABLE;
	_canData.TxData[2] = 0x00;
	if(HAL_CAN_AddTxMessage(_hcan, &_canData.TxHeader, _canData.TxData, &_canData.TxMailbox) != HAL_OK)	Error_Handler();
	_inverter->state = OFF_MODE;
	osDelay(INVERTER_DELAY);
}

void activateInverter(CAN_HandleTypeDef *_hcan, InverterData *_inverter){

	deactivateInverter(_hcan,_inverter);
	CANData _canData;
	_canData.TxHeader.IDE = CAN_ID_STD;
	_canData.TxHeader.StdId = INVERTER_TX_CAN_ID;
	_canData.TxHeader.RTR = CAN_RTR_DATA;
	_canData.TxHeader.DLC = 3;
	_canData.TxData[0] = INVERTER_CONTROL_MODE_COMMAND;
	_canData.TxData[1] = INVERTER_ENABLE;
	_canData.TxData[2] = 0x00;
	if(HAL_CAN_AddTxMessage(_hcan, &_canData.TxHeader, _canData.TxData, &_canData.TxMailbox) != HAL_OK)	Error_Handler();
	osDelay(INVERTER_DELAY);
	HAL_GPIO_WritePin(_inverter->run.group, _inverter->run.pin, GPIO_PIN_SET);
	_inverter->state = RUN_MODE;
	return;
}



