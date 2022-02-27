/*
 * UART.c
 *
 *  Created on: Dec 17, 2021
 *      Author: patricio
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"


/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @defgroup RTC_Calendar
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO ITStatus UartRXReady;
extern __IO ITStatus UartTXReady;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*********************************************************************
   * USART1 Init
   *
   */


void USART_Init(UART_HandleTypeDef* pUartHandle){
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
        - Word Length = 8 Bits
        - Stop Bit = One Stop bit
        - Parity = None
        - BaudRate = 9600 baud
        - Hardware flow control disabled (RTS and CTS signals) */
  pUartHandle->Instance        = USARTx;

  pUartHandle->Init.BaudRate   = USART_SPEED;
  pUartHandle->Init.WordLength = UART_WORDLENGTH_8B;
  pUartHandle->Init.StopBits   = UART_STOPBITS_1;
  pUartHandle->Init.Parity     = UART_PARITY_NONE;
  pUartHandle->Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  pUartHandle->Init.Mode       = UART_MODE_TX_RX;
  //pUartHandle->Init.Mode       = UART_MODE_TX;
  if(HAL_UART_DeInit(pUartHandle) != HAL_OK)
  {
    //Error_Handler();
  }
  if(HAL_UART_Init(pUartHandle) != HAL_OK)
  {
    //Error_Handler();
  }
}



/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  if (UartHandle->Instance == USARTx) {
    UartTXReady = SET;
  }


  /* Turn LED6 on: Transfer in transmission process is correct */
  //BSP_LED_On(LED6);
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  if (UartHandle->Instance == USARTx) {
    UartRXReady = SET;
  }

  /* Turn LED4 on: Transfer in reception process is correct */
  //BSP_LED_On(LED4);
}


/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  /* Turn LED3 on: Transfer error in reception/transmission process */
  //BSP_LED_On(LED3);
}






