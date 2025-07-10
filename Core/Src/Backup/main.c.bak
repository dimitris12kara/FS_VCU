/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
VCUData vcu;
ACUData acu;
InverterData inverter;
IsabellenData ivt;
DataLoggerData dataLogger;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ErrorHandlerRaiseErrorFlag = 0;
uint16_t counterFilter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t page = 2;
uint8_t EEPROMtxData[1]={0x12};
uint8_t id = 4;
uint8_t EEPROMrxData[2];
HAL_StatusTypeDef eepromStatus;

uint8_t write_data[20];
uint8_t read_data[20];
uint32_t i, status;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM8_Init();
  MX_CAN2_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_ADC_Start_DMA(&hadc2, (uint32_t *)vcu.rawSensorData, 3) != HAL_OK) Error_Handler();

  if(VCUInit(&vcu, &START_STATE_TIMER, &ADC_SAMPLE_RATE_TIMER, BRAKELIGHT_THRESHOLD, R2D_BRAKE_THRESHOLD) != HAL_OK) Error_Handler();// VCU Structure data initialization

  if(DataloggerInit(&dataLogger) != HAL_OK) Error_Handler();// Datalogger Structure data initialization

  if(IVTInit(&ivt) != HAL_OK) Error_Handler();// IVT Structure data initialization

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t flag5= 0;
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if(htim->Instance ==  vcu.ADCSampleRateTimer.timer->Instance){ //Analog Filtering. GAMAEI
	  flag5++;

//	  vcu.brakeSensor.sum = vcu.brakeSensor.sum + vcu.rawSensorData[2] - vcu.brakeSensor.sampleArray[counterFilter];
//	  vcu.APPS1.sum = vcu.APPS1.sum + vcu.rawSensorData[1] - vcu.APPS1.sampleArray[counterFilter];
//	  vcu.APPS2.sum = vcu.APPS2.sum + vcu.rawSensorData[0] - vcu.APPS2.sampleArray[counterFilter];
//
//	  vcu.brakeSensor.filtered = vcu.brakeSensor.sum / MOVING_AVERAGE;
//	  vcu.APPS1.filtered = vcu.APPS1.sum / MOVING_AVERAGE;
//	  vcu.APPS2.filtered = vcu.APPS2.sum / MOVING_AVERAGE;
//
//	  vcu.brakeSensor.sampleArray[counterFilter] = vcu.rawSensorData[2];
//	  vcu.APPS1.sampleArray[counterFilter] = vcu.rawSensorData[1];
//	  vcu.APPS2.sampleArray[counterFilter] = vcu.rawSensorData[0];
//
//	  counterFilter++;
//	  if(counterFilter >= MOVING_AVERAGE) counterFilter = 0;
  }
  else if (htim->Instance == vcu.startStateTimer.timer->Instance) vcu.startStateTimer.triggered = TIMER_TRIGGERED;
  else if(htim->Instance == inverter.timeOutTimer.timer->Instance){
	  inverter.isConnected = 0;
	  inverter.timeOutTimer.triggered = TIMER_TRIGGERED;
  }
  else if(htim->Instance == acu.SCSLEDsTimeOutTimer.timer->Instance){
	  acu.SCSLEDsTimeOutTimer.triggered = TIMER_TRIGGERED;
	  acu.AIRMinusIsArmed = 0;
	  acu.AIRPlusIsArmed = 0;
	  acu.PREIsArmed = 0;
  }


  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	ErrorHandlerRaiseErrorFlag++;

  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
