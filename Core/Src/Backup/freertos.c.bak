/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iwdg.h"
#include "utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float flag = 0;

extern ACUData acu;
extern InverterData inverter;
extern IsabellenData ivt;
extern DataLoggerData dataLogger;
extern VCUData vcu;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for state */
osThreadId_t stateHandle;
const osThreadAttr_t state_attributes = {
  .name = "state",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for serialBars */
osThreadId_t serialBarsHandle;
const osThreadAttr_t serialBars_attributes = {
  .name = "serialBars",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for serialData */
osThreadId_t serialDataHandle;
const osThreadAttr_t serialData_attributes = {
  .name = "serialData",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for displayComm */
osSemaphoreId_t displayCommHandle;
const osSemaphoreAttr_t displayComm_attributes = {
  .name = "displayComm"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void stateTask(void *argument);
void serialBarsTask(void *argument);
void serialDataTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= 1;
	DWT->CYCCNT = 0;
}

__weak unsigned long getRunTimeCounterValue(void)
{
return DWT->CYCCNT;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of displayComm */
  displayCommHandle = osSemaphoreNew(1, 0, &displayComm_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of state */
  stateHandle = osThreadNew(stateTask, NULL, &state_attributes);

  /* creation of serialBars */
  serialBarsHandle = osThreadNew(serialBarsTask, NULL, &serialBars_attributes);

  /* creation of serialData */
  serialDataHandle = osThreadNew(serialDataTask, NULL, &serialData_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_stateTask */
uint8_t buttonR2D = 0;
/**
  * @brief  Function implementing the state thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_stateTask */
void stateTask(void *argument)
{
  /* USER CODE BEGIN stateTask */
  CANData DataLoggerCanData;
  // Inverter Structure data initialization
  osDelay(10);

  osDelay(1000);

  if (InverterInit(&inverter, &criticalCan, &INVERTER_TIMEOUT_TIMER) != HAL_OK) Error_Handler();
  if (ACUInit(&acu, &SCS_TIMEOUT_TIMER, SCS_ACCEPTED_ERRORS) != HAL_OK) Error_Handler();

  // ACU Structure data initialization

  //Reads Data from eeprom everytime LVMS has power.
  retrieveVCUDataFromEepromOnStartUp(&eepromI2C, &vcu);

  // Enable CallBack functions as late as possible to avoid hardfaults with RAM and not correct initialization of structures.
  if (HAL_CAN_ActivateNotification(&sensorsCan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)  Error_Handler();

  // Enables Independant Watchdog Timer. After that function, It waits for the refresh command, or else the mcu will reset.
  MX_IWDG_Init();
  /* Infinite loop */
  for(;;)
  {
	  osDelay(1);
	  if(vcu.startStateTimer.triggered){
		vcu.startStateTimer.triggered = TIMER_RESET;
		devicesTimeOutTimers(&vcu, &acu, &dataLogger, &ivt);
		// Checks whether some devices are connected correctly to CANBuses.

		HAL_IWDG_Refresh(&hiwdg);// Refreshes the Independant Watchdog Timer.
		// Checks for SCS implausibillities.May be implemented with hardware.
		if(acu.SCSLEDsTimeOutTimer.triggered || acu.SCSLEDsCheckSumCurrentErrors > acu.SCSLEDsCheckSumAcceptedErrors){
			//If an SCS Error has occured, lights must indicate its own failure.
			HAL_GPIO_WritePin(acu.IMDLed.group, acu.IMDLed.pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(acu.AMSLed.group, acu.AMSLed.pin, GPIO_PIN_SET);
		}
		else{
			// If none SCS Error has occured, the state of the light comes from ACU via Critical CANBus.
			HAL_GPIO_WritePin(acu.IMDLed.group, acu.IMDLed.pin, acu.IMDHasError);
			HAL_GPIO_WritePin(acu.AMSLed.group, acu.AMSLed.pin, acu.AMSHasLatchedError);
		}

		pedalSensor(&vcu);// Updates final value of APPS. Checks for SCS Errors.
		brakeSensor(&vcu);// Updates final value of BrakeSensor. Checks for SCS Errors.


		if(1){//Sends pedalBox data to the DataLogger capture only if we have enabled them from the menu page.
			DataLoggerCanData.TxHeader.IDE = CAN_ID_STD;
			DataLoggerCanData.TxHeader.StdId = PEDALS_TX_CAN_ID;
			DataLoggerCanData.TxHeader.RTR = CAN_RTR_DATA;
			DataLoggerCanData.TxHeader.DLC = 7;
			DataLoggerCanData.TxData[0] = vcu.pedalSensorFinalValue >> 8; // MSB of APPS final value
			DataLoggerCanData.TxData[1] = vcu.pedalSensorFinalValue;// LSB of APPS final value
			DataLoggerCanData.TxData[2] = (int16_t)(vcu.brakePressure * 100) >> 8;// MSB of BrakeSensor final value
			DataLoggerCanData.TxData[3] = (int16_t)(vcu.brakePressure * 100);// LSB of BrakeSensor final value
			DataLoggerCanData.TxData[4] = vcu.state;
			DataLoggerCanData.TxData[4] |= (vcu.pedalSensorFinalValue < -10) << 2;
			DataLoggerCanData.TxData[4] |= (vcu.brakeSensorFinalValue < -10) << 3;
			DataLoggerCanData.TxData[4] |= (inverter.isConnected) << 4;
			DataLoggerCanData.TxData[4] |= vcu.lastError << 5;
			DataLoggerCanData.TxData[5] = (uint8_t)map(inverter.igbtTemp, 10, 60, 20, 100);
			DataLoggerCanData.TxData[6] = (uint8_t)map(inverter.motorTemp, 10, 60, 20, 100);
			if(HAL_CAN_AddTxMessage(&sensorsCan, &DataLoggerCanData.TxHeader, DataLoggerCanData.TxData, &DataLoggerCanData.TxMailbox) != HAL_OK) Error_Handler();
		}

		// Controls BrakeLight. May need to play with MOVING_AVERAGE constant in order to minimize the delay of the light.
		if(vcu.brakeSensorFinalValue > vcu.brakeLightThreshold) HAL_GPIO_WritePin(vcu.brakeLight.group, vcu.brakeLight.pin, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(vcu.brakeLight.group, vcu.brakeLight.pin, GPIO_PIN_RESET);
		// Switch that handles the state of the racecar.
		switch(vcu.state){
		case OFF://First state that comes when shutdown is opened or first state after LVMS is powered.
			if(vcu.offStateFirstLoopEnter){
				vcu.offStateFirstLoopEnter = ENTERED;
				setTorque(&criticalCan, 0);// Sets commanded torque of the inverter to zero.

			  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
			    changeTextColor2Screen(&displayUart, "t0", "BLACK");// Changes color of Mode Displayed in Nextion Display.
				osSemaphoreRelease(displayCommHandle);

			    osDelay(DISPLAY_DELAY);

			  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
			    str2Screen(&displayUart, "t0", "Mode:Off");// Changes text of Mode displayed in Nextion Display.
				osSemaphoreRelease(displayCommHandle);

			    deactivateInverter(&criticalCan, &inverter);// Disables the inverter through CANBus and Analog Signal (RUN).
				continue;//Will iterate again from the main for(;;)
			}
			// If precharge is done succesfully, the relays will close and the VCU will transition to the next state (PRECHARGED)
			if(acu.AIRMinusIsArmed && acu.AIRPlusIsArmed && acu.PREIsArmed){
				vcu.offStateFirstLoopEnter = TO_ENTER;
				vcu.state = PRECHARGED;// Changes state of the racecar.

			  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
				error2Screen(&displayUart, "Precharge done succesfully!");
				osSemaphoreRelease(displayCommHandle);
				continue;
			}
		break;
		case PRECHARGED:// Second state that indicates precharge has been completed succesfully, and R2D is not possible, so no commanded torque is applied to the motor controller
			// If shutdown opens, return to first state of the racecar.
			if(!acu.AIRMinusIsArmed || !acu.AIRPlusIsArmed || !acu.PREIsArmed){
				vcu.prechargedStateFirstLoopEnter = TO_ENTER;
				vcu.state = OFF;
			  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
				error2Screen(&displayUart, "Shutdown opened!");
				osSemaphoreRelease(displayCommHandle);
				continue;
			}

			if(vcu.prechargedStateFirstLoopEnter){
				vcu.R2DButton.state = NOT_PRESSED;
				vcu.prechargedStateFirstLoopEnter = ENTERED;

				setTorque(&criticalCan, 0);// Sets commanded torque of the inverter to zero.

			  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
			    changeTextColor2Screen(&displayUart, "t0", "RED");
				osSemaphoreRelease(displayCommHandle);

			    osDelay(DISPLAY_DELAY);

			  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
			    str2Screen(&displayUart, "t0", "Mode:PRE");
				osSemaphoreRelease(displayCommHandle);

			    deactivateInverter(&criticalCan, &inverter);// Disables the inverter through CANBus and Analog Signal (RUN).
				continue;
			}
			if(vcu.R2DButton.state && !vcu.prechargedStateFirstLoopEnter){
				//If R2D Button is pressed.
				vcu.R2DButton.state = NOT_PRESSED;
				if(vcu.brakeSensorFinalValue > vcu.brakeR2DThreshold){
				//And if the Brakesensor value is above a threshold.
					vcu.prechargedStateFirstLoopEnter = TO_ENTER;
					vcu.state = R2D;
					// If both previous conditions are true, only then preceed to third state of the racecar, Ready To Drive,
					// in which commanded torque through APPS will be sent to inverter

				  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
					error2Screen(&displayUart, "Going mallia!");
					osSemaphoreRelease(displayCommHandle);
					continue;

				}
			}
		break;
		case R2D:
			// If shutdown opens, return to first state of the racecar.
//			if(!acu.AIRPlusIsArmed || !(acu.PREIsArmed || acu.AIRMinusIsArmed)){
			if(!acu.AIRMinusIsArmed || !acu.AIRPlusIsArmed || !acu.PREIsArmed){
				vcu.R2DStateFirstLoopEnter = TO_ENTER;
				vcu.state = OFF;
			  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
				error2Screen(&displayUart, "Shutdown opened!");
				osSemaphoreRelease(displayCommHandle);
				continue;
			}
			// If the R2D button is pressed when in R2D Mode, the racecar will transition to its previous state.
			if(vcu.R2DButton.state && !vcu.R2DStateFirstLoopEnter){
				vcu.R2DButton.state = NOT_PRESSED;
				vcu.R2DStateFirstLoopEnter = TO_ENTER;
				vcu.state = PRECHARGED;
			  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
				error2Screen(&displayUart, "Returning to Pre");
				osSemaphoreRelease(displayCommHandle);
				continue;
			}
			// Ιf inverter timeout happen, return to previous mode of the racecar.
			if(inverter.timeOutTimer.triggered){
				vcu.R2DStateFirstLoopEnter = TO_ENTER;
				vcu.state = PRECHARGED;
				vcu.lastError = INVERTER_DISCONNECTED;
			  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
				error2Screen(&displayUart, "Inverter Comm Error");
				osSemaphoreRelease(displayCommHandle);
				continue;
			}
			// Ιf APPS implausibillities happen, return to previous mode of the racecar.
			if(vcu.pedalSensorFinalValue < 0){
				vcu.R2DStateFirstLoopEnter = TO_ENTER;
				vcu.state = PRECHARGED;
			  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
				error2Screen(&displayUart, "Accel pedal Error");
				osSemaphoreRelease(displayCommHandle);
				continue;
			}

			// Ιf BrakeSensor implausibillities happen, return to previous mode of the racecar.
			if(vcu.brakeSensorFinalValue < 0){
				vcu.R2DStateFirstLoopEnter = TO_ENTER;
				vcu.state = PRECHARGED;
			  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
				error2Screen(&displayUart, "Brake Error");
				osSemaphoreRelease(displayCommHandle);
				continue;
			}

#ifdef SOFTWARE_BSPD
///////////START OF POUTSA SEQUENCE
			if(vcu.pedalSensorFinalValue > 0.25 * ADC_MAX_VALUE && vcu.brakeSensorFinalValue > 0.2 * ADC_MAX_VALUE){
				setTorque(&criticalCan, 0);
			    error2Screen(&displayUart, "APPS 25%, Release accel pedal");
			    while((vcu.pedalSensorFinalValue > 0.05 * ADC_MAX_VALUE || vcu.brakeSensorFinalValue > 0.1 * ADC_MAX_VALUE) && vcu.state == R2D){
					osDelay(40);
					setTorque(&criticalCan, 0);
			    	devicesTimeOutTimers(&vcu, &acu, &dataLogger, &ivt);// Checks whether some devices are connected correctly to CANBuses.
					HAL_IWDG_Refresh(&hiwdg);// Refreshes the Independant Watchdog Timer.

					// Checks for SCS implausibillities.May be implemented with hardware.
					if(acu.SCSLEDsTimeOutTimer.triggered || acu.SCSLEDsCheckSumCurrentErrors > acu.SCSLEDsCheckSumAcceptedErrors){
						//If an SCS Error has occured, lights must indicate its own failure.
						HAL_GPIO_WritePin(acu.IMDLed.group, acu.IMDLed.pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(acu.AMSLed.group, acu.AMSLed.pin, GPIO_PIN_SET);
					}
					else{
						// If none SCS Error has occured, the state of the light comes from ACU via Critical CANBus.
						HAL_GPIO_WritePin(acu.IMDLed.group, acu.IMDLed.pin, acu.IMDHasError);
						HAL_GPIO_WritePin(acu.AMSLed.group, acu.AMSLed.pin, acu.AMSHasLatchedError);
					}

					pedalSensor(&vcu);// Updates final value of APPS. Checks for SCS Errors.
					brakeSensor(&vcu);// Updates final value of BrakeSensor. Checks for SCS Errors.

					if(1){//Sends pedalBox data to the DataLogger capture only if we have enabled them from the menu page.
						DataLoggerCanData.TxHeader.IDE = CAN_ID_STD;
						DataLoggerCanData.TxHeader.StdId = PEDALS_TX_CAN_ID;
						DataLoggerCanData.TxHeader.RTR = CAN_RTR_DATA;
						DataLoggerCanData.TxHeader.DLC = 7;
						DataLoggerCanData.TxData[0] = vcu.pedalSensorFinalValue >> 8; // MSB of APPS final value
						DataLoggerCanData.TxData[1] = vcu.pedalSensorFinalValue;// LSB of APPS final value
						DataLoggerCanData.TxData[2] = vcu.brakeSensorFinalValue >> 8;// MSB of BrakeSensor final value
						DataLoggerCanData.TxData[3] = vcu.brakeSensorFinalValue;// LSB of BrakeSensor final value
						DataLoggerCanData.TxData[4] = vcu.state;
						DataLoggerCanData.TxData[4] |= (vcu.pedalSensorFinalValue < -10) << 2;
						DataLoggerCanData.TxData[4] |= (vcu.brakeSensorFinalValue < -10) << 3;
						DataLoggerCanData.TxData[4] |= (inverter.isConnected) << 4;
						DataLoggerCanData.TxData[4] |= vcu.lastError << 5;
						DataLoggerCanData.TxData[5] = (uint8_t)map(inverter.igbtTemp, 10, 60, 20, 100);
						DataLoggerCanData.TxData[6] = (uint8_t)map(inverter.motorTemp, 10, 60, 20, 100);

						if(HAL_CAN_AddTxMessage(&sensorsCan, &DataLoggerCanData.TxHeader, DataLoggerCanData.TxData, &DataLoggerCanData.TxMailbox) != HAL_OK) Error_Handler();
					}

					// Controls BrakeLight. May need to play with MOVING_AVERAGE constant in order to minimize the delay of the light.
					if(vcu.brakeSensorFinalValue > vcu.brakeLightThreshold) HAL_GPIO_WritePin(vcu.brakeLight.group, vcu.brakeLight.pin, GPIO_PIN_SET);
					else HAL_GPIO_WritePin(vcu.brakeLight.group, vcu.brakeLight.pin, GPIO_PIN_RESET);

					if(!acu.AIRMinusIsArmed || !acu.AIRPlusIsArmed || !acu.PREIsArmed){
						vcu.R2DStateFirstLoopEnter = TO_ENTER;
						vcu.state = OFF;

					  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
						error2Screen(&displayUart, "Shutdown opened!");
						osSemaphoreRelease(displayCommHandle);
						continue;
					}
					// If the R2D button is pressed when in R2D Mode, the racecar will transition to its previous state.
					if(vcu.R2DButton.state && !vcu.R2DStateFirstLoopEnter){
						vcu.R2DButton.state = NOT_PRESSED;
						vcu.R2DStateFirstLoopEnter = TO_ENTER;
						vcu.state = PRECHARGED;
					  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
						error2Screen(&displayUart, "Returning to Pre");
						osSemaphoreRelease(displayCommHandle);
						continue;
					}
					// Ιf inverter timeout happen, return to previous mode of the racecar.
					if(inverter.timeOutTimer.triggered){
						vcu.R2DStateFirstLoopEnter = TO_ENTER;
						vcu.state = PRECHARGED;
						vcu.lastError = INVERTER_DISCONNECTED;
					  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
						error2Screen(&displayUart, "Inverter Comm Error");
						osSemaphoreRelease(displayCommHandle);
						continue;

					}
					// Ιf APPS implausibillities happen, return to previous mode of the racecar.
					if(vcu.pedalSensorFinalValue < 0){
						vcu.R2DStateFirstLoopEnter = TO_ENTER;
						vcu.state = PRECHARGED;
					  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
						error2Screen(&displayUart, "Accel pedal Error");
						osSemaphoreRelease(displayCommHandle);
						continue;
					}

					// Ιf BrakeSensor implausibillities happen, return to previous mode of the racecar.
					if(vcu.brakeSensorFinalValue < 0){
						vcu.R2DStateFirstLoopEnter = TO_ENTER;
						vcu.state = PRECHARGED;
					  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
						error2Screen(&displayUart, "Brake Error");
						osSemaphoreRelease(displayCommHandle);
						continue;
					}

			    }
				if(vcu.state == R2D) error2Screen(&displayUart, "APPS at 5%");
				continue;
			}
///////////END OF POUTSA SEQUENCE
#endif
			// If none error happen during the first iteration in R2D Mode, it will enter this condition in which Buzzer is enabled and inverter is disabled
			if(vcu.R2DStateFirstLoopEnter){
				vcu.R2DStateFirstLoopEnter = ENTERED;
				osDelay(100);

			  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
			    changeTextColor2Screen(&displayUart, "t0", "BLUE");
				osSemaphoreRelease(displayCommHandle);

			    osDelay(DISPLAY_DELAY);

			    osSemaphoreAcquire(displayCommHandle, osWaitForever);
			    str2Screen(&displayUart, "t0", "Mode:R2D");
				osSemaphoreRelease(displayCommHandle);

			    HAL_GPIO_WritePin(vcu.buzzer.group, vcu.buzzer.pin, GPIO_PIN_SET);//Buzzer sound started.
			    osDelay(BUZZER_DELAY);// Buzzer duration should be between 1 and 3 seconds. (FS RULES 2024)
			    HAL_GPIO_WritePin(vcu.buzzer.group, vcu.buzzer.pin, GPIO_PIN_RESET);//Buzzer sound ended.
			    osDelay(INVERTER_DELAY);
			    activateInverter(&criticalCan, &inverter);//Activates Inverter both through CANBus and analog signal(RUN).
				continue;
			}
//		if(vcu.pedalSensorFinalValue > 0) HAL_GPIO_WritePin(inverter.run.group, inverter.run.pin, GPIO_PIN_SET);
//		else HAL_GPIO_WritePin(inverter.run.group, inverter.run.pin, GPIO_PIN_RESET);
		setTorque(&criticalCan, map(vcu.pedalSensorFinalValue, 0, 4095, 0, vcu.maxTorque));// If none error happen, sends commanded torque of the APPS to Motor Controller.
//		setTorque(&criticalCan, vcu.pedalSensorFinalValue);// If none error happen, sends commanded torque of the APPS to Motor Controller.
		break;
		default:
		break;
		}
		}
	}
  /* USER CODE END stateTask */
}

/* USER CODE BEGIN Header_serialBarsTask */
/**
* @brief Function implementing the serialBars thread. It updates the Serial Bars of the Nextion diplay.
* @param argument: Not used
* @retval None
*/


/* USER CODE END Header_serialBarsTask */
void serialBarsTask(void *argument)
{
  /* USER CODE BEGIN serialBarsTask */
  //osSemaphoreRelease(displayCommHandle);

  /* Infinite loop */
  for(;;){
	osDelay(100 - DISPLAY_DELAY);
	if(vcu.menuPage == MENU_PAGE) continue;

	 flag = flag + 1;
	 if(flag > 100) flag = 0;

  	osSemaphoreAcquire(displayCommHandle, osWaitForever);
  	if(vcu.pedalSensorFinalValue < 0) bar2Screen(&displayUart, "j0", 0);
  	else bar2Screen(&displayUart, "j0", (int)map(vcu.pedalSensorFinalValue, 0, 4095, 0, 100));
	osSemaphoreRelease(displayCommHandle);

	osDelay(DISPLAY_DELAY);

	osSemaphoreAcquire(displayCommHandle, osWaitForever);
	if(vcu.brakeSensorFinalValue < 0) bar2Screen(&displayUart, "j1", 0);
	else bar2Screen(&displayUart, "j1", (int)map(vcu.brakeSensorFinalValue, 0, 4095, 0, 100));
	osSemaphoreRelease(displayCommHandle);

  }
  /* USER CODE END serialBarsTask */
}

/* USER CODE BEGIN Header_serialDataTask */
/**
* @brief Function that cahnges and updates valus in the nextion display.
* @param argument: Not used
* @retval None
*/
uint8_t rxData[4];

/* USER CODE END Header_serialDataTask */
void serialDataTask(void *argument)
{
  /* USER CODE BEGIN serialDataTask */
  osSemaphoreRelease(displayCommHandle);
  uint8_t buf[10];
  CANData canData;
  char time[10];

  /* Infinite loop */
  for(;;)
  {

	osDelay(1000 - 7 * DISPLAY_DELAY);
	if(acu.AIRsAreStuck)
		error2Screen(&displayUart, "Vges apo to amaksi dike mou");

//	if(ivt.current > 8.33 && vcu.brakePressure > 30)
//		error2Screen(&displayUart, "BSPD Error");
//	sprintf((char*)time,"%02d:%02d", dataLogger.hours, dataLogger.minutes);
//	str2Screen(&displayUart, "t10", time);

	switch(vcu.menuPage){
		case MAIN_PAGE:
			if(vcu.loadNewScreen){

				vcu.loadNewScreen = 0;

//			writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID + 14, (uint8_t *)vcu.menuPage, 1);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			changePage2Screen(&displayUart, "0");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);


			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t11", "Main");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t1", "Vdc");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t2", "Motor");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t3", "Speed");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t4", "H Cell");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t5", "I");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t7", "IGBT");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t8", "Pwr");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t9", "Bat");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);
		}
		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x1", ivt.voltage1);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x2", inverter.motorTemp);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart,"x3", dataLogger.vehicleSpeed);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x4", acu.highestCellTemp);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x5", (float)ivt.current);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x7", inverter.igbtTemp);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x8", ivt.wattage);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x9", 100.0 * ivt.voltage1 / MAX_PACK_VOLTAGE);
		osSemaphoreRelease(displayCommHandle);
	break;

	case ELECTRICAL_PAGE:
		if(vcu.loadNewScreen){

			vcu.loadNewScreen = ENTERED;

//			writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID + 14, (uint8_t *)vcu.menuPage, 1);
			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t11", "Elec");
			osSemaphoreRelease(displayCommHandle);


			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t1", "I AC");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t2", "IMD R");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t3", "I");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t4", "ICnt");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t5", "Pwr");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t7", "PwrCnt:");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t8", "MinV");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t9", "MaxV");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);
		}
		//V bus
		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x1", inverter.actualCurrent);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);
		// IMD Res
		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x2", acu.IMDResistance);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x3", (float)ivt.current);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x4", ivt.currentCounter);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x5", ivt.wattage);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x7", ivt.wattageCounter);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x8", acu.minVoltageCell.val * 10);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x9", acu.maxVoltageCell.val * 10);
		osSemaphoreRelease(displayCommHandle);



	break;

	case MECHANICAL_PAGE:
		//Speed
		//Accel
		//max accel
		//
		if(vcu.loadNewScreen){
			vcu.loadNewScreen = ENTERED;

//			writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID + 14, (uint8_t *)vcu.menuPage, 1);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t11", "Mech");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t1", "FL RPM");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t2", "FR RPM");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t3", "RL RPM");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t4", "RR RPM");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t5", "Hum");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t7", "BrPre");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t8", "INV RPM");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t9", "/Ratio");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

		}
		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x1", dataLogger.frontLeftWheelRPM);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x2", dataLogger.frontRightWheelRPM);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x3", dataLogger.rearLeftWheelRPM);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x4", dataLogger.rearRightWheelRPM);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x5", acu.humidity);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x7", vcu.brakePressure);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x8", inverter.RPMSpeed);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x9", inverter.RPMSpeed / 3.9705);
		osSemaphoreRelease(displayCommHandle);


	break;

	case TEMPERATURES_PAGE:
		if(vcu.loadNewScreen){

			vcu.loadNewScreen = ENTERED;

//			writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID + 14, (uint8_t *)vcu.menuPage, 1);


			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t11", "Temps");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t1", "Motor");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t2", "IGBT");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t3", "H Cell");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t4", "Inv H");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t5", "Mtr F");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t7", "Inv C");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t8", "Vicor");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t9", "ACU");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

		}
		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x1", inverter.motorTemp);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x2", inverter.igbtTemp);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x3", acu.highestCellTemp);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x4", dataLogger.coolingThermistors[4]);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x5", dataLogger.coolingThermistors[1]);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x7", dataLogger.coolingThermistors[3]);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x8", acu.vicorTemperature);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x9", acu.PCBTemperature);
		osSemaphoreRelease(displayCommHandle);








	break;

	case DEBUG_PAGE:
		if(vcu.loadNewScreen){

			vcu.loadNewScreen = ENTERED;

//			writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID + 14, (uint8_t *)vcu.menuPage, 1);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t11", "Debug");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t1", "Plus");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t2", "Minus");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t3", "Pre");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t4", "Flag");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t5", "60V");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t7", "IMDE");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t8", "AMSE");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t9", "AIRsE");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

		}
		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x1", acu.AIRPlusIsArmed);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x2", acu.AIRMinusIsArmed);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x3", acu.PREIsArmed);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x4", flag);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x5", acu.TSOver60Volt);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x7", acu.IMDHasError);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x8", acu.AMSHasError);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x9", acu.AIRsAreStuck);
		osSemaphoreRelease(displayCommHandle);

	break;

	case DEVICES_PAGE:
		if(vcu.loadNewScreen){

			vcu.loadNewScreen = ENTERED;

//			writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID + 14, (uint8_t *)vcu.menuPage, 1);


			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t11", "Devices");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t1", "ACU");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t2", "IVT");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t3", "INV");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t4",  "Main");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t5", "CBox");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t7", "Aero");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t8", "Tele");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

			osSemaphoreAcquire(displayCommHandle, osWaitForever);
			str2Screen(&displayUart, "t9", "User");
			osSemaphoreRelease(displayCommHandle);

			osDelay(DISPLAY_DELAY);

		}
		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x1", acu.isConnected);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x2", ivt.isConnected);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x3", inverter.isConnected);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x4", dataLogger.mainIsConnected);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x5", dataLogger.controlBoxIsConnected);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x7", dataLogger.aeroIsConnected);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x8", vcu.telemetryIsConnected);
		osSemaphoreRelease(displayCommHandle);

		osDelay(DISPLAY_DELAY);

		osSemaphoreAcquire(displayCommHandle, osWaitForever);
		int2Screen(&displayUart, "x9", acu.userIsConnected);
		osSemaphoreRelease(displayCommHandle);

	break;
	case MENU_PAGE:
//		writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID + 14, (uint8_t *)vcu.menuPage, 1);
		changePage2Screen(&displayUart, "4");
		HAL_UART_Receive(&displayUart, rxData, 4, 100);
	    bar2Screen(&displayUart, "Menu.h0", map(vcu.maxTorque, 0, 4095, 0, 100));
		while(vcu.menuPage == MENU_PAGE){
			osDelay(1);
			if(HAL_UART_Receive(&displayUart, rxData, 4, 100) != HAL_OK) continue;

				switch(rxData[2]){

				case 101: // Accel Remapping
		            error2Screen(&displayUart, "MENU when Accel released");
		            vcu.utils2Button.state = NOT_PRESSED;
		            vcu.menuPage = MAIN_PAGE;
		            while(vcu.utils2Button.state == NOT_PRESSED) osDelay(10);
		            vcu.utils2Button.state = NOT_PRESSED;
		            vcu.menuPage = MAIN_PAGE;
		            vcu.APPS1.mappedMin = vcu.rawSensorData[1] - MAPPED_THRESHOLD_OFFSET;
		            vcu.APPS2.mappedMin = vcu.rawSensorData[0] - MAPPED_THRESHOLD_OFFSET;
		            buf[0] = vcu.APPS1.mappedMin >> 8;
		            buf[1] = vcu.APPS1.mappedMin;
		            buf[2] = vcu.APPS2.mappedMin >> 8;
		            buf[3] = vcu.APPS2.mappedMin;
		            writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID, buf, 4);

		            osDelay(1000);

		            error2Screen(&displayUart, "MENU when Accel fully pressed");
		            vcu.utils2Button.state = NOT_PRESSED;
		            while(vcu.utils2Button.state == NOT_PRESSED) osDelay(10);
		            vcu.utils2Button.state = NOT_PRESSED;
		            vcu.menuPage = MAIN_PAGE;
		            vcu.APPS1.mappedMax = vcu.rawSensorData[1] + MAPPED_THRESHOLD_OFFSET;
		            vcu.APPS2.mappedMax = vcu.rawSensorData[0] + MAPPED_THRESHOLD_OFFSET;
		            buf[0] = vcu.APPS1.mappedMax >> 8;
		            buf[1] = vcu.APPS1.mappedMax;
		            buf[2] = vcu.APPS2.mappedMax >> 8;
		            buf[3] = vcu.APPS2.mappedMax;

		            writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID + 4, buf, 4);
					vcu.menuPage = MAIN_PAGE;
		            vcu.loadNewScreen = TO_ENTER;

				break;

				case 102:
		            error2Screen(&displayUart, "MENU when Brake fully released");
		            vcu.utils2Button.state = NOT_PRESSED;
		            vcu.menuPage = MAIN_PAGE;
		            while(vcu.utils2Button.state == NOT_PRESSED) osDelay(10);
		            vcu.utils2Button.state = NOT_PRESSED;
		            vcu.brakeSensor.mappedMin = vcu.rawSensorData[2] - MAPPED_THRESHOLD_OFFSET;

		            buf[0] = vcu.brakeSensor.mappedMin >> 8;
		            buf[1] = vcu.brakeSensor.mappedMin;
		            writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID + 8, buf, 2);

		            osDelay(1000);

		            error2Screen(&displayUart, "MENU when Brake fully pressed");
		            vcu.utils2Button.state = NOT_PRESSED;
		            while(vcu.utils2Button.state == NOT_PRESSED) osDelay(10);
		            vcu.menuPage = MAIN_PAGE;
		            vcu.utils2Button.state = NOT_PRESSED;
		            vcu.brakeSensor.mappedMax = vcu.rawSensorData[2] + MAPPED_THRESHOLD_OFFSET;
		            buf[0] = vcu.brakeSensor.mappedMax >> 8;
		            buf[1] = vcu.brakeSensor.mappedMax;
		            writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID + 10, buf, 2);
					vcu.menuPage = MAIN_PAGE;
					vcu.loadNewScreen = TO_ENTER;

				break;

				case 103:// Eeprom inverter in eco mode!
				break;

				case 104://Spare Button 1
				break;

				case 105://Spare Button 2
					vcu.APPS1.mappedMin = APPS1_MIN_DEFAULT;
					vcu.APPS1.mappedMax = APPS1_MAX_DEFAULT;
					vcu.APPS2.mappedMin = APPS2_MIN_DEFAULT;
					vcu.APPS2.mappedMax = APPS2_MAX_DEFAULT;
					vcu.brakeSensor.mappedMin = BRAKESENSOR_MIN_DEFAULT;
					vcu.brakeSensor.mappedMax = BRAKESENSOR_MAX_DEFAULT;
					buf[0] = vcu.APPS1.mappedMax >> 8;
					buf[1] = vcu.APPS1.mappedMax;
					buf[2] = vcu.APPS2.mappedMax >> 8;
					buf[3] = vcu.APPS2.mappedMax;
					buf[4] = vcu.brakeSensor.mappedMax >> 8;
					buf[5] = vcu.brakeSensor.mappedMax;

		            writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID, buf, 6);
					vcu.menuPage = MAIN_PAGE;
					vcu.loadNewScreen = TO_ENTER;




				break;

				case 106:// Eeprom inverter in sport mode!
				break;

				case 107:
				break;
				case 108:
					dataLogger.arePoweredOn = 1;
					canData.TxHeader.IDE = CAN_ID_STD;
					canData.TxHeader.DLC = 1;
					canData.TxHeader.StdId = DATALOGGER_ACTIVATION_TX_CAN_ID;
					canData.TxData[0] = 1;
					for(int i = 0; i < 2; i++) 	if(HAL_CAN_AddTxMessage(&criticalCan, &canData.TxHeader, canData.TxData, &canData.TxMailbox) != HAL_OK) Error_Handler();
					changeTextColor2Screen(&displayUart, "t10", "RED");
					vcu.menuPage = MAIN_PAGE;
					vcu.loadNewScreen = TO_ENTER;
				break;

				case 109:
					vcu.menuPage = MAIN_PAGE;
					vcu.loadNewScreen = TO_ENTER;
				break;


//				case 0:
//				break;


				default: //Max torque is changed
					vcu.maxTorque = map(rxData[2], 0, 100, 0, 4095);
					buf[0] = vcu.maxTorque >> 8;
					buf[1] = vcu.maxTorque;
		            writeDataEeprom(&eepromI2C, EEPROM_STARTING_ID + 12, buf, 2);

				break;

				}


		}

	break;
	default:

	break;

	}
  }

  /* USER CODE END serialDataTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

