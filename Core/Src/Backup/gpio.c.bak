/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "utils.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R2D_Output_Pin|R2D_Output_LED_Pin|CAN2_LED_Pin|CAN1_LED_Pin
                          |BrakeControl_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Programmable_LED_2_Pin|Programmable_LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMD_LED_GPIO_Port, IMD_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AMS_LED_GPIO_Port, AMS_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BuzzerControl_Pin|NOT_WC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : R2D_Output_Pin R2D_Output_LED_Pin CAN2_LED_Pin CAN1_LED_Pin
                           BrakeControl_Pin IMD_LED_Pin */
  GPIO_InitStruct.Pin = R2D_Output_Pin|R2D_Output_LED_Pin|CAN2_LED_Pin|CAN1_LED_Pin
                          |BrakeControl_Pin|IMD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Programmable_LED_2_Pin Programmable_LED_1_Pin */
  GPIO_InitStruct.Pin = Programmable_LED_2_Pin|Programmable_LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MENU_2_Button_Pin R2D_Button_Pin TSact_Button_Pin */
  GPIO_InitStruct.Pin = MENU_2_Button_Pin|R2D_Button_Pin|TSact_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MENU_1_Button_Pin */
  GPIO_InitStruct.Pin = MENU_1_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MENU_1_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AMS_LED_Pin */
  GPIO_InitStruct.Pin = AMS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AMS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BuzzerControl_Pin NOT_WC_Pin */
  GPIO_InitStruct.Pin = BuzzerControl_Pin|NOT_WC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

//	if(vcu.R2DButton.pin == GPIO_Pin) {
//		if(vcu.state != OFF) vcu.R2DButton.state = PRESSED;
//	}
	if(vcu.R2DButton.pin == GPIO_Pin){
		if(vcu.state == R2D && !vcu.R2DStateFirstLoopEnter){
			vcu.R2DButton.state = PRESSED;
		}
		else if(vcu.state == PRECHARGED && !vcu.prechargedStateFirstLoopEnter){
			vcu.R2DButton.state = PRESSED;
		}


	}
	else if (vcu.tsActButton.pin == GPIO_Pin) vcu.tsActButton.state = PRESSED;

	else if (vcu.utils1Button.pin == GPIO_Pin && vcu.loadNewScreen == ENTERED){
		vcu.loadNewScreen = TO_ENTER;
		vcu.menuPage++;
		if(vcu.menuPage > MENU_PAGE - 1) vcu.menuPage = MAIN_PAGE;
		vcu.utils1Button.state = PRESSED;
	}
	else if (vcu.utils2Button.pin == GPIO_Pin){
		if(vcu.state != OFF) return;
		vcu.utils2Button.state = PRESSED;

//		vcu.loadNewScreen = TO_ENTER;
		if(vcu.menuPage == MENU_PAGE) {
			vcu.menuPage = MAIN_PAGE;
			vcu.loadNewScreen = TO_ENTER;
		}
		else vcu.menuPage = MENU_PAGE;

}
}
/* USER CODE END 2 */
