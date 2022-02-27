/**
  ******************************************************************************
  * @file    BSP/Inc/main.h 
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
//#include "stdio.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_ts.h"
#include "stm32f429i_discovery_io.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_gyroscope.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#ifdef EE_M24LR64
#include "stm32f429i_discovery_eeprom.h"
#endif /*EE_M24LR64*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  void   (*DemoFunc)(void);
  uint8_t DemoName[50]; 
  uint32_t DemoIndex;
}BSP_DemoTypedef;

extern const unsigned char stlogo[];
extern const uint16_t hisa[];

// Return codes from the functions in this class. Remember to update GetStatusCodeName() if you add more.
typedef enum StatusCode {
	STATUS_OK				= 1,	// Success
	STATUS_ERROR			= 2,	// Error in communication
	STATUS_COLLISION		= 3,	// Collission detected
	STATUS_TIMEOUT			= 4,	// Timeout in communication.
	STATUS_NO_ROOM			= 5,	// A buffer is not big enough.
	STATUS_INTERNAL_ERROR	= 6,	// Internal error in the code. Should not happen ;-)
	STATUS_INVALID			= 7,	// Invalid argument.
	STATUS_CRC_WRONG		= 8,	// The CRC_A does not match
	STATUS_MIFARE_NACK		= 9,	// A MIFARE PICC responded with NAK.
	STATUS_UID_COMPLETE		= 0xA,	// UID complete, PICC compliant with ISO/IEC 14443-4
	STATUS_UID_COMPLETE_NC	= 0xB,	// UID complete, PICC not compliant with ISO/IEC 14443-4
	STATUS_UID_NOTCOMPLETE	= 0xC,	// Cascade bit set: UID not complete
	STATUS_INV_COLL_POS		= 0xD,	// Invalid collision position
	STATUS_AUTH_FAILED		= 0xE,	// Invalid collision position
	STATUS_MIFARE_W_FAIL	= 0xF,	// Invalid collision position
	STATUS_MIFARE_R_FAIL	= 0x10	// Invalid collision position
} StatusCode_t;

typedef uint8_t  	byte; 		// 1byte
typedef uint16_t  	word; 		// 2bytes
typedef uint32_t  	dword; 		// 4bytes
typedef enum {false, true} bool;

/* Exported constants --------------------------------------------------------*/
#define USART_SPEED                   115200
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __USART1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART1

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_STREAM              DMA1_Stream6
#define USARTx_RX_DMA_STREAM              DMA1_Stream5
#define USARTx_TX_DMA_CHANNEL             DMA_CHANNEL_4
#define USARTx_RX_DMA_CHANNEL             DMA_CHANNEL_4


/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Stream6_IRQn
#define USARTx_DMA_RX_IRQn                DMA1_Stream5_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Stream6_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA1_Stream5_IRQHandler

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

/* Definition for SPIx clock resources */
#define SPIx                             SPI4
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI4_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOE_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI4_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI4_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_GPIO_PORT               	 GPIOE
#define SPIx_SCK_PIN                     GPIO_PIN_2
#define SPIx_SCK_GPIO_PORT               GPIOE
#define SPIx_SCK_AF                      GPIO_AF5_SPI4
#define SPIx_MISO_PIN                    GPIO_PIN_5
#define SPIx_MISO_GPIO_PORT              GPIOE
#define SPIx_MISO_AF                     GPIO_AF5_SPI4
#define SPIx_MOSI_PIN                    GPIO_PIN_6
#define SPIx_MOSI_GPIO_PORT              GPIOE
#define SPIx_MOSI_AF                     GPIO_AF5_SPI4

#define SPIx_NSS_GPIO_PORT               GPIOE
#define SPIx_NSS_PIN                     GPIO_PIN_3

/* Definition for SPIx's NVIC */
#define SPIx_IRQn                        SPI4_IRQn
#define SPIx_IRQHandler                  SPI4_IRQHandler


/* Definition of GPIO pins */
#define RELAY_PORT               	 	 GPIOB
#define RELAY_IN_PIN               	 	 GPIO_PIN_4


#define SIZE4 4
#define NUM_CODES_SIZE4 5
#define SIZE7 7
#define NUM_CODES_SIZE7 1
#define SIZE16 16
#define NUM_CODES_SIZE16 1

/* Exported macro ------------------------------------------------------------*/

#define INT_ROUND(x)        ((x)>=0?(int32_t)((x)+0.5):(int32_t)((x)-0.5))
#define UINT_ROUND(x)       (uint32_t)((x)+0.5)
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

#define COUNT_OF_EXAMPLE(x)    (sizeof(x)/sizeof(BSP_DemoTypedef))
/* Exported functions ------------------------------------------------------- */

HAL_StatusTypeDef InitSPI (SPI_HandleTypeDef* SpiHandle);
uint8_t SPI_TransmitByte(SPI_HandleTypeDef *hspi, uint8_t data);
void 	SPI_ReceiveByte(SPI_HandleTypeDef *hspi, uint8_t *pData);
void USART_Init(UART_HandleTypeDef* pUartHandle);

void Joystick_demo (void);
void Touchscreen_demo (void);
void LCD_demo (void);
void MEMS_demo (void);
void Log_demo(void);
#ifdef EE_M24LR64
void EEPROM_demo (void);
#endif /*EE_M24LR64*/
void SD_demo (void);
void Touchscreen_Calibration (void);
uint16_t Calibration_GetX(uint16_t x);
uint16_t Calibration_GetY(uint16_t y);
uint8_t IsCalibrationDone(void);
uint8_t CheckForUserInput(void);
void Toggle_Leds(void);
void GPIOA_Init(void);
void GPIOB_Init(void);
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
