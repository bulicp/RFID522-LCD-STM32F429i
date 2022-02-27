/*
 * SPI.c
 *
 *  Created on: Jan 7, 2022
 *      Author: patriciobulic
 */

#include "main.h"


HAL_StatusTypeDef InitSPI (SPI_HandleTypeDef* SpiHandle){

	// Set the SPI parameters
	SpiHandle->Instance               = SPIx;
	SpiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	SpiHandle->Init.Direction         = SPI_DIRECTION_2LINES;
	SpiHandle->Init.CLKPhase          = SPI_PHASE_1EDGE;
	SpiHandle->Init.CLKPolarity       = SPI_POLARITY_LOW;
	SpiHandle->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	SpiHandle->Init.CRCPolynomial     = 10;
	SpiHandle->Init.DataSize          = SPI_DATASIZE_8BIT;
	SpiHandle->Init.FirstBit          = SPI_FIRSTBIT_MSB;
	SpiHandle->Init.NSS               = SPI_NSS_SOFT;
	SpiHandle->Init.TIMode            = SPI_TIMODE_DISABLE;
	SpiHandle->Init.Mode 			  = SPI_MODE_MASTER;


	if(HAL_SPI_Init(SpiHandle) != HAL_OK)
	{
		//Error_Handler();
		return HAL_ERROR;
	}
	return HAL_OK;
}


/**
  * @brief  TxRx Transfer completed callback
  * @param  hspi: SPI handle.
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	// TODO if needed
}

/**
  * @brief  SPI error callbacks
  * @param  hspi: SPI handle
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	 // TODO if needed
}




/**
  * @brief  SPI transmit byte in full duplex mode
  * @param  hspi: SPI handle, data - trasmit data
  * @retval recv data
  */
uint8_t SPI_TransmitByte(SPI_HandleTypeDef *hspi, uint8_t data){
	uint8_t TxData[1];
	uint8_t RxData[1];

	TxData[0] = data;
	HAL_SPI_TransmitReceive(hspi, TxData, RxData, 1, HAL_MAX_DELAY);

	return RxData[0];
}

/* OBSOLETE, PB.

void SPI_ReceiveByte(SPI_HandleTypeDef *hspi, uint8_t *pRxData){
	uint8_t TxData[1] = {0x00}; //dummy

	HAL_SPI_TransmitReceive(hspi, TxData, pRxData, 1, HAL_MAX_DELAY);
}
*/





