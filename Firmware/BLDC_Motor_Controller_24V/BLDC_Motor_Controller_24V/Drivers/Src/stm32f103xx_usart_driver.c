/*
 * stm32f103xx_usart_driver.c
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#include "stm32f103xx.h"


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

static void UART_DMATransmitCplt(DMA_HandleTypeDef *pDMAHandle);
static void UART_DMATxHalfCplt(DMA_HandleTypeDef *pDMAHandle);
static void UART_DMAError(DMA_HandleTypeDef *pDMAHandle);



void USART_Init(UART_HandleTypeDef *pUSARTHandle)
{
	// Temporary register for USART configuration
	uint32_t config = 0;

	// Init the Low level hardware of USART : GPIO, CLOCK
	USART_MspInit(pUSARTHandle);

	// USARTx Disable for configuration
	USART_PeripheralControl(pUSARTHandle->Instance, DISABLE);

	/* --------------------- USART_CR1 configuration --------------------- */

	// 1. Enable USART Tx and Rx engines according to the Mode configuration item
	config |= (uint32_t)pUSARTHandle->Init.Mode | pUSARTHandle->Init.WordLength | pUSARTHandle->Init.Parity;
	MODIFY_REG(pUSARTHandle->Instance->CR1, (uint32_t)(USART_CR1_TE | USART_CR1_RE | USART_CR1_M | USART_CR1_PCE | USART_CR1_PS), config);
	config = 0;

	/* --------------------- USART_CR2 configuration --------------------- */
	config |= (uint32_t)pUSARTHandle->Init.StopBits;
	MODIFY_REG(pUSARTHandle->Instance->CR2, (uint32_t)USART_CR2_STOP, config);
	config = 0;

	/* --------------------- USART_CR3 configuration --------------------- */
	config |= (uint32_t)pUSARTHandle->Init.HwFlowCtl;
	MODIFY_REG(pUSARTHandle->Instance->CR3, (uint32_t)(USART_CR3_CTSE | USART_CR3_RTSE), config);
	config = 0;

	/* --------------------- USART_BRR configuration --------------------- */
	USART_SetBaudRate(pUSARTHandle->Instance, pUSARTHandle->Init.BaudRate);


	// USARTx Enable for configuration
	USART_PeripheralControl(pUSARTHandle->Instance, ENABLE);

	// Init the USART state
	pUSARTHandle->State = USART_STATE_READY;
}



__weak void USART_MspInit(UART_HandleTypeDef *pUSARTHandle)
{
	/* Prevent unused argument(s) compilation warning */
		UNUSED(pUSARTHandle);

	/* NOTE : This function should not be modified, when the callback is needed,
	 * 		  the USART_MspInit could be implemented in the user file
	 * 		  (This is a weak implementation. The user application may override this function)
	 */
}



void USART_PeripheralClockControl(USART_TypeDef *USARTx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(USARTx == USART1)		RCC_USART1_CLK_ENABLE();
		else if(USARTx == USART2)	RCC_USART2_CLK_ENABLE();
		else if(USARTx == USART3)	RCC_USART3_CLK_ENABLE();
		else if(USARTx == UART4)	RCC_UART4_CLK_ENABLE();
		else if(USARTx == UART5)	RCC_UART5_CLK_ENABLE();
	}
	else if(En_or_Di == DISABLE)
	{
		if(USARTx == USART1)		RCC_USART1_CLK_DISABLE();
		else if(USARTx == USART2)	RCC_USART2_CLK_DISABLE();
		else if(USARTx == USART3)	RCC_USART3_CLK_DISABLE();
		else if(USARTx == UART4)	RCC_UART4_CLK_DISABLE();
		else if(USARTx == UART5)	RCC_UART5_CLK_DISABLE();
	}
}


void USART_PeripheralControl(USART_TypeDef *USARTx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)				SET_BIT(USARTx->CR1, USART_CR1_UE);
	else if(En_or_Di == DISABLE)		CLEAR_BIT(USARTx->CR1, USART_CR1_UE);
}


void USART_SetBaudRate(USART_TypeDef *USARTx, uint32_t BaudRate)
{
	uint32_t pclk;
	uint32_t usartDiv;
	uint32_t mantissa, fraction;
	uint8_t tmp;

	// 1. Get PCLK of USARTx
	if(USARTx == USART1)
	{
		// Case of USART1
		pclk = RCC_GetPCLKxValue(PCLK2);
	}
	else
	{
		// Case of USART2/3/4/5
		pclk = RCC_GetPCLKxValue(PCLK1);
	}

	// 2. Calculate USARTDIV vlaue by given PCLK, BaudRate
	usartDiv = ((float)pclk / (16 * BaudRate)) * 100;

	// 3. Extract Mantissa part from USARTDIV
	mantissa = usartDiv / 100;

	// 4. Extract Fraction part from USARTDIV
	tmp = usartDiv - (mantissa * 100);

	fraction = (uint8_t)((((tmp * 16) + 50) / 100) & 0xF);

	// 5. Write Mantissa part and Fraction part value to USART_BRR
	CLEAR_REG(USARTx->BRR);

	USARTx->BRR |= ((mantissa << 4) | fraction);
}


void USART_Transmit(UART_HandleTypeDef *pUSARTHandle, uint8_t *TxBuffer, uint16_t Size)
{
	// 1. Check that state of USARTx is READY
	if(pUSARTHandle->State != USART_STATE_READY)
	{
		// state of USARTx is not ready
		return;
	}

	// 2. Change state of USARTx to BUSY_TX
	pUSARTHandle->State = USART_STATE_BUSY_TX;

	// 3. Configure TxBufferSize, TxCount of USART Handler
	pUSARTHandle->TxXferSize = Size;
	pUSARTHandle->TxXferCount = Size;

	// 4. Write data to USART_DR until TxCount value reaches to zero
	while(pUSARTHandle->TxXferCount > 0)
	{
		WAIT_FLAG_SET(pUSARTHandle->Instance->SR, USART_SR_TXE);

		pUSARTHandle->Instance->DR = (uint8_t)(*TxBuffer & 0xFF);

		TxBuffer++;
		pUSARTHandle->TxXferCount--;
	}

	// 5. Wait until TC flag is set in the SR
	WAIT_FLAG_SET(pUSARTHandle->Instance->SR, USART_SR_TC);

	// 6. Change state of USARTx to READY
	pUSARTHandle->State = USART_STATE_READY;
}


uint8_t USART_Transmit_IT(UART_HandleTypeDef *pUSARTHandle, uint8_t *TxBuffer, uint16_t Size)
{
	// 1. Check that state of USARTx is READY
	if(pUSARTHandle->State != USART_STATE_READY)
	{
		// state of USARTx is not ready
		return pUSARTHandle->State;
	}

	// 2. Change state of USARTx to BUSY_TX
	pUSARTHandle->State = USART_STATE_BUSY_TX;

	// 3. Configure TxBufferPointer, TxBufferSize, TxCount of USART Handler
	pUSARTHandle->pTxBuffPtr = TxBuffer;
	pUSARTHandle->TxXferSize = Size;
	pUSARTHandle->TxXferCount = Size;

	// 4. Enable USART TXE, TC interrupt
	SET_BIT(pUSARTHandle->Instance->CR1, USART_CR1_TXEIE);
	SET_BIT(pUSARTHandle->Instance->CR1, USART_CR1_TCIE);

	return pUSARTHandle->State;
}


void UART_Transmit_DMA(UART_HandleTypeDef *pUSARTHandle, uint8_t *pData, uint16_t Size)
{
	uint32_t *tmp;

	pUSARTHandle->pTxBuffPtr = pData;
	pUSARTHandle->TxXferSize = Size;
	pUSARTHandle->TxXferCount = Size;

    /* Set the UART DMA transfer complete callback */
	pUSARTHandle->hdmatx->XferCpltCallback = UART_DMATransmitCplt;

    /* Set the UART DMA Half transfer complete callback */
	pUSARTHandle->hdmatx->XferHalfCpltCallback = UART_DMATxHalfCplt;

    /* Set the DMA error callback */
	pUSARTHandle->hdmatx->XferErrorCallback = UART_DMAError;

    /* Set the DMA abort callback */
	pUSARTHandle->hdmatx->XferAbortCallback = NULL;

	/* Enable the UART transmit DMA channel */
	tmp = (uint32_t *)&pData;
	DMA_Start_IT(pUSARTHandle->hdmatx, *(uint32_t *)tmp, (uint32_t)&pUSARTHandle->Instance->DR, Size);

	/* Clear the TC flag in the SR register by writing 0 to it */
	CLEAR_FLAG(pUSARTHandle->Instance->SR, UART_FLAG_TC);

	/* Enable the DMA transfer for transmit request by setting the DMAT bit
	   in the UART CR3 register */
	SET_BIT(pUSARTHandle->Instance->CR3, USART_CR3_DMAT);
}


__weak void UART_TxCpltCallback(UART_HandleTypeDef *pUSARTHandle)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(pUSARTHandle);
	/* NOTE: This function should not be modified, when the callback is needed,
		   the UART_TxCpltCallback could be implemented in the user file
	*/
}


__weak void UART_TxHalfCpltCallback(UART_HandleTypeDef *pUSARTHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pUSARTHandle);
  /* NOTE: This function should not be modified, when the callback is needed,
           the UART_TxHalfCpltCallback could be implemented in the user file
   */
}


__weak void UART_ErrorCallback(UART_HandleTypeDef *pUSARTHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pUSARTHandle);
  /* NOTE: This function should not be modified, when the callback is needed,
           the UART_ErrorCallback could be implemented in the user file
   */
}


__weak void USART_ApplicationEventCallback(UART_HandleTypeDef *pUSARTHandle, uint8_t AppEV)
{
	/* Prevent unused argument(s) compilation warning */
		UNUSED(pUSARTHandle);
		UNUSED(AppEV);

	/* NOTE : This function should not be modified, when the callback is needed,
	 * 		  the USART_MspInit could be implemented in the user file
	 * 		  (This is a weak implementation. The user application may override this function)
	 */
}


void USART_IRQHandling(UART_HandleTypeDef *pUSARTHandle)
{
	/* Interrupt handling for USART */

	uint32_t temp1, temp2;

	// 1. Handle for interrupt generated by TC event
	temp1 = READ_BIT(pUSARTHandle->Instance->SR, USART_SR_TC);
	temp2 = READ_BIT(pUSARTHandle->Instance->CR1, USART_CR1_TCIE);

	if(temp1 && temp2)
	{
		// This interrupt is generated by TC event

		// In this block, Close transmission and call application callback if TxCount is zero
		if(pUSARTHandle->State == USART_STATE_BUSY_TX)
		{
			// Check the TxCount. If it is zero, then close the data transmission
			if(pUSARTHandle->TxXferCount == 0)
			{
				// Clear the TC flag
				CLEAR_BIT(pUSARTHandle->Instance->SR, USART_SR_TC);

				// Clear the TCIE flag (Disable interrupt for TC flag)
				CLEAR_BIT(pUSARTHandle->Instance->CR1, USART_CR1_TCIE);

				// Reset the application state
				pUSARTHandle->State = USART_STATE_READY;

				// Reset TxBufferPointer to NULL
				pUSARTHandle->pTxBuffPtr = NULL;

				// Reset the TxBufferSize, TxCount to zero
				pUSARTHandle->TxXferSize = 0;
				pUSARTHandle->TxXferCount = 0;

				// Call the application callback with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	// 2. Handle for interrupt generated by TXE event
	temp1 = READ_BIT(pUSARTHandle->Instance->SR, USART_SR_TXE);
	temp2 = READ_BIT(pUSARTHandle->Instance->CR1, USART_CR1_TXEIE);

	if(temp1 && temp2)
	{
		// This interrupt is generated by TXE event

		if(pUSARTHandle->State == USART_STATE_BUSY_TX)
		{
			// Keep transmitting data until TxCount reaches to zero
			if(pUSARTHandle->TxXferCount > 0)
			{
				pUSARTHandle->Instance->DR = (uint8_t)(*pUSARTHandle->pTxBuffPtr & 0xFF);

				pUSARTHandle->pTxBuffPtr++;
				pUSARTHandle->TxXferCount--;
			}

			if(pUSARTHandle->TxXferCount == 0)
			{
				// TxCount is zero
				// Clear the TXEIE bit (Disable interrupt for TXE flag)
				CLEAR_BIT(pUSARTHandle->Instance->CR1, USART_CR1_TXEIE);
			}

		}

	}

}



static void UART_DMATransmitCplt(DMA_HandleTypeDef *pDMAHandle)
{
	UART_HandleTypeDef *pUSARTHandle = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)pDMAHandle)->Parent;

	UART_TxCpltCallback(pUSARTHandle);
}


static void UART_DMATxHalfCplt(DMA_HandleTypeDef *pDMAHandle)
{
	UART_HandleTypeDef *pUSARTHandle = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)pDMAHandle)->Parent;

	UART_TxHalfCpltCallback(pUSARTHandle);
}


static void UART_DMAError(DMA_HandleTypeDef *pDMAHandle)
{
	UART_HandleTypeDef *pUSARTHandle = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)pDMAHandle)->Parent;

	UART_ErrorCallback(pUSARTHandle);
}


