/*
 * GPIO.c
 *
 *  Created on: Dec 28, 2021
 *      Author: patricio
 */

#include "main.h"

void GPIOB_Init(void) {
  GPIO_InitTypeDef  GPIO_InitStruct;

  __HAL_RCC_GPIOB_CLK_ENABLE();


  GPIO_InitStruct.Pin = RELAY_IN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Disable Relay input at init
  HAL_GPIO_WritePin(GPIOB, RELAY_IN_PIN, GPIO_PIN_SET);

}

void GPIOA_Init(void) {
  GPIO_InitTypeDef  GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();


  GPIO_InitStruct.Pin = RELAY_IN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Disable Relay input at init
  HAL_GPIO_WritePin(GPIOA, RELAY_IN_PIN, GPIO_PIN_SET);
}



