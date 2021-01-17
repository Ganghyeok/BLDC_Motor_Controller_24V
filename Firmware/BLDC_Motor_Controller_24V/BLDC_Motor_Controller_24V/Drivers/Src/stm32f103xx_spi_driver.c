/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Jan 16, 2021
 *      Author: Ganghyeok Lim
 */

#include "stm32f103xx.h"


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void SPI_Init(SPI_HandleTypeDef *pSPIHandle)
{
	SPI_MspInit(pSPIHandle);

	SPI_DISABLE(pSPIHandle);

	/*----------------------- SPIx CR1 & CR2 Configuration ---------------------*/
	/* Configure : SPI Mode, Communication Mode, Data size, Clock polarity and phase, NSS management,
	Communication speed, First bit and CRC calculation state */
	WRITE_REG(pSPIHandle->Instance->CR1, ((pSPIHandle->Init.Mode & (SPI_CR1_MSTR | SPI_CR1_SSI)) |
								  	  	  (pSPIHandle->Init.Direction & (SPI_CR1_RXONLY | SPI_CR1_BIDIMODE)) |
										  	 (pSPIHandle->Init.DataSize & SPI_CR1_DFF) |
											 (pSPIHandle->Init.CLKPolarity & SPI_CR1_CPOL) |
											 (pSPIHandle->Init.CLKPhase & SPI_CR1_CPHA) |
											 (pSPIHandle->Init.NSS & SPI_CR1_SSM) |
											 (pSPIHandle->Init.BaudRatePrescaler & SPI_CR1_BR_Msk) |
											 (pSPIHandle->Init.FirstBit  & SPI_CR1_LSBFIRST) |
											 (pSPIHandle->Init.CRCCalculation & SPI_CR1_CRCEN)));

	/* Configure : NSS management */
	WRITE_REG(pSPIHandle->Instance->CR2, ((pSPIHandle->Init.NSS >> 16U) & SPI_CR2_SSOE));
}


__weak void SPI_MspInit(SPI_HandleTypeDef *pSPIHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pSPIHandle);

  /* NOTE : This function should not be modified, when the callback is needed,
            the SPI_MspInit should be implemented in the user file
   */
}


void SPI_PeripheralClockControl(SPI_TypeDef *SPIx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(SPIx == SPI1)		RCC_SPI1_CLK_ENABLE();
		else if(SPIx == SPI2)	RCC_SPI2_CLK_ENABLE();
		else if(SPIx == SPI3)	RCC_SPI3_CLK_ENABLE();
	}
	else if(En_or_Di == DISABLE)
	{
		if(SPIx == SPI1)		RCC_SPI1_CLK_DISABLE();
		else if(SPIx == SPI2)	RCC_SPI2_CLK_DISABLE();
		else if(SPIx == SPI3)	RCC_SPI3_CLK_DISABLE();
	}
}


void SPI_Transmit(SPI_HandleTypeDef *pSPIHandle, uint8_t *pData, uint16_t Size)
{
	uint16_t initial_TxXferCount;

	initial_TxXferCount = Size;

	/* Set the transaction information */
	pSPIHandle->State = SPI_STATE_BUSY_TX;
	pSPIHandle->pTxBuffPtr = (uint8_t *)pData;
	pSPIHandle->TxXferSize = Size;
	pSPIHandle->TxXferCount = Size;

	/* Init field not used in handle to zero */
	pSPIHandle->pRxBuffPtr = (uint8_t *)NULL;
	pSPIHandle->RxXferSize = 0U;
	pSPIHandle->RxXferCount = 0U;
	pSPIHandle->TxISR = NULL;
	pSPIHandle->RxISR = NULL;

	/* Configure communication direction : 1 Line */
	if(pSPIHandle->Init.Direction == SPI_DIRECTION_1LINE)
	{
		/* Disable SPI Peripheral before set 1 Line direction (BIDIOE bit) */
		SPI_DISABLE(pSPIHandle);
		SPI_1LINE_TX(pSPIHandle);
	}

	/* Check if the SPI is already enabled */
	if((pSPIHandle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
		/* Enable SPI peripheral */
		SPI_ENABLE(pSPIHandle);
	}


	/* Transmit data in 16 Bit mode */
	if(pSPIHandle->Init.DataSize == SPI_DATASIZE_16BIT)
	{
		if((pSPIHandle->Init.Mode == SPI_MODE_SLAVE) || (initial_TxXferCount == 0x01U))
		{
			pSPIHandle->Instance->DR = *((uint16_t *)pSPIHandle->pTxBuffPtr);
			pSPIHandle->pTxBuffPtr += sizeof(uint16_t);
			pSPIHandle->TxXferCount--;
		}

		/* Transmit data in 16 Bit mode */
		while(pSPIHandle->TxXferCount > 0U)
		{
			/* Wait until TXE flag is set to send data */
			if(SPI_GET_FLAG(pSPIHandle, SPI_FLAG_TXE))
			{
				pSPIHandle->Instance->DR = *((uint16_t *)pSPIHandle->pTxBuffPtr);
				pSPIHandle->pTxBuffPtr += sizeof(uint16_t);
				pSPIHandle->TxXferCount--;
			}
		}
	}


	/* Transmit data in 8 Bit mode */
	else
	{
		if((pSPIHandle->Init.Mode == SPI_MODE_SLAVE) || (initial_TxXferCount == 0x01U))
		{
			*((__IO uint8_t *)&pSPIHandle->Instance->DR) = (*pSPIHandle->pTxBuffPtr);
			pSPIHandle->pTxBuffPtr += sizeof(uint8_t);
			pSPIHandle->TxXferCount--;
		}
		while(pSPIHandle->TxXferCount > 0U)
		{
			/* Wait until TXE flag is set to send data */
			if(SPI_GET_FLAG(pSPIHandle, SPI_FLAG_TXE))
			{
				*((__IO uint8_t *)&pSPIHandle->Instance->DR) = (*pSPIHandle->pTxBuffPtr);
				pSPIHandle->pTxBuffPtr += sizeof(uint8_t);
				pSPIHandle->TxXferCount--;
			}
		}
	}

	if(pSPIHandle->Init.Direction == SPI_DIRECTION_2LINES)
	{
		SPI_CLEAR_OVRFLAG(pSPIHandle);
	}

	pSPIHandle->State = SPI_STATE_READY;
}


void SPI_Receive(SPI_HandleTypeDef *pSPIHandle, uint8_t *pData, uint16_t Size)
{
	if ((pSPIHandle->Init.Mode == SPI_MODE_MASTER) && (pSPIHandle->Init.Direction == SPI_DIRECTION_2LINES))
	{
		pSPIHandle->State = SPI_STATE_BUSY_RX;
		/* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
		return SPI_TransmitReceive(pSPIHandle, pData, pData, Size);
	}

	/* Set the transaction information */
	pSPIHandle->State = SPI_STATE_BUSY_RX;
	pSPIHandle->pRxBuffPtr = (uint8_t *)pData;
	pSPIHandle->RxXferSize = Size;
	pSPIHandle->RxXferCount = Size;

	/*Init field not used in handle to zero */
	pSPIHandle->pTxBuffPtr = (uint8_t *)NULL;
	pSPIHandle->TxXferSize = 0U;
	pSPIHandle->TxXferCount = 0U;
	pSPIHandle->TxISR = NULL;
	pSPIHandle->RxISR = NULL;

	/* Configure communication direction: 1Line */
	if(pSPIHandle->Init.Direction == SPI_DIRECTION_1LINE)
	{
		/* Disable SPI Peripheral before set 1Line direction (BIDIOE bit) */
		SPI_DISABLE(pSPIHandle);
		SPI_1LINE_RX(pSPIHandle);
	}

	/* Check if the SPI is already enabled */
	if((pSPIHandle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
		/* Enable SPI peripheral */
		SPI_ENABLE(pSPIHandle);
	}

	/* Receive data in 8 Bit mode */
	if(pSPIHandle->Init.DataSize == SPI_DATASIZE_8BIT)
	{
		/* Transfer loop */
		while(pSPIHandle->RxXferCount > 0U)
		{
			/* Check the RXNE flag */
			if(SPI_GET_FLAG(pSPIHandle, SPI_FLAG_RXNE))
			{
				/* read the received data */
				(* (uint8_t *)pSPIHandle->pRxBuffPtr) = *(__IO uint8_t *)&pSPIHandle->Instance->DR;
				pSPIHandle->pRxBuffPtr += sizeof(uint8_t);
				pSPIHandle->RxXferCount--;
			}
		}
	}
	else
	{
		/* Transfer loop */
		while(pSPIHandle->RxXferCount > 0U)
		{
			/* Check the RXNE flag */
			if(SPI_GET_FLAG(pSPIHandle, SPI_FLAG_RXNE))
			{
				*((uint16_t *)pSPIHandle->pRxBuffPtr) = (uint16_t)pSPIHandle->Instance->DR;
				pSPIHandle->pRxBuffPtr += sizeof(uint16_t);
				pSPIHandle->RxXferCount--;
			}
		}
	}

	pSPIHandle->State = SPI_STATE_READY;
}


void SPI_TransmitReceive(SPI_HandleTypeDef *pSPIHandle, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	uint16_t initial_TxXferCount;
	uint32_t tmp_mode;
	uint32_t tmp_state;
	uint32_t txallowed = 1U;

	/* Init temporary variables */
	tmp_state = pSPIHandle->State;
	tmp_mode = pSPIHandle->Init.Mode;
	initial_TxXferCount = Size;

	UNUSED(tmp_mode);
	UNUSED(tmp_state);

	/* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
	if (pSPIHandle->State != SPI_STATE_BUSY_RX)
	{
		pSPIHandle->State = SPI_STATE_BUSY_TX_RX;
	}

	/* Set the transaction information */
	pSPIHandle->pRxBuffPtr = (uint8_t *)pRxData;
	pSPIHandle->RxXferCount = Size;
	pSPIHandle->RxXferSize = Size;
	pSPIHandle->pTxBuffPtr = (uint8_t *)pTxData;
	pSPIHandle->TxXferCount = Size;
	pSPIHandle->TxXferSize = Size;

	/*Init field not used in handle to zero */
	pSPIHandle->RxISR = NULL;
	pSPIHandle->TxISR = NULL;

	/* Check if the SPI is already enabled */
	if((pSPIHandle->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
		/* Enable SPI peripheral */
		SPI_ENABLE(pSPIHandle);
	}

	/* Transmit and Receive data in 16 Bit mode */
	if(pSPIHandle->Init.DataSize == SPI_DATASIZE_16BIT)
	{
		if((pSPIHandle->Init.Mode == SPI_MODE_SLAVE) || (initial_TxXferCount == 0x01U))
		{
			pSPIHandle->Instance->DR = *((uint16_t *)pSPIHandle->pTxBuffPtr);
			pSPIHandle->pTxBuffPtr += sizeof(uint16_t);
			pSPIHandle->TxXferCount--;
		}
		while((pSPIHandle->TxXferCount > 0U) || (pSPIHandle->RxXferCount > 0U))
		{
			/* Check TXE flag */
			if((SPI_GET_FLAG(pSPIHandle, SPI_FLAG_TXE)) && (pSPIHandle->TxXferCount > 0U) && (txallowed == 1U))
			{
				pSPIHandle->Instance->DR = *((uint16_t *)pSPIHandle->pTxBuffPtr);
				pSPIHandle->pTxBuffPtr += sizeof(uint16_t);
				pSPIHandle->TxXferCount--;

				/* Next Data is a reception (Rx). Tx not allowed */
				txallowed = 0U;
			}

			/* Check RXNE flag */
			if((SPI_GET_FLAG(pSPIHandle, SPI_FLAG_RXNE)) && (pSPIHandle->RxXferCount > 0U))
			{
				*((uint16_t *)pSPIHandle->pRxBuffPtr) = (uint16_t)pSPIHandle->Instance->DR;
				pSPIHandle->pRxBuffPtr += sizeof(uint16_t);
				pSPIHandle->RxXferCount--;

				/* Next Data is a Transmission (Tx). Tx is allowed */
				txallowed = 1U;
			}
		}
	}

	/* Transmit and Receive data in 8 Bit mode */
	else
	{
		if((pSPIHandle->Init.Mode == SPI_MODE_SLAVE) || (initial_TxXferCount == 0x01U))
		{
			*((__IO uint8_t *)&pSPIHandle->Instance->DR) = (*pSPIHandle->pTxBuffPtr);
			pSPIHandle->pTxBuffPtr += sizeof(uint8_t);
			pSPIHandle->TxXferCount--;
		}
		while((pSPIHandle->TxXferCount > 0U) || (pSPIHandle->RxXferCount > 0U))
		{
			/* Check TXE flag */
			if((SPI_GET_FLAG(pSPIHandle, SPI_FLAG_TXE)) && (pSPIHandle->TxXferCount > 0U) && (txallowed == 1U))
			{
				*(__IO uint8_t *)&pSPIHandle->Instance->DR = (*pSPIHandle->pTxBuffPtr);
				pSPIHandle->pTxBuffPtr++;
				pSPIHandle->TxXferCount--;

				/* Next Data is a reception (Rx). Tx not allowed */
				txallowed = 0U;
			}

			/* Wait until RXNE flag is reset */
			if((SPI_GET_FLAG(pSPIHandle, SPI_FLAG_RXNE)) && (pSPIHandle->RxXferCount > 0U))
			{
				(*(uint8_t *)pSPIHandle->pRxBuffPtr) = pSPIHandle->Instance->DR;
				pSPIHandle->pRxBuffPtr++;
				pSPIHandle->RxXferCount--;

				/* Next Data is a Transmission (Tx). Tx is allowed */
				txallowed = 1U;
			}
		}
	}

	if(pSPIHandle->Init.Direction == SPI_DIRECTION_2LINES)
	{
		SPI_CLEAR_OVRFLAG(pSPIHandle);
	}

	pSPIHandle->State = SPI_STATE_READY;
}







