/*
 * stm32f103xx_dma_driver.c
 *
 *  Created on: 2021. 1. 7.
 *      Author: Ganghyeok Lim
 */

#include "stm32f103xx.h"


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void DMA_Init(DMA_HandleTypeDef *pDMAHandle)
{
	uint32_t config = 0;

	// 1. Calculate the channel index
	if ((uint32_t)(pDMAHandle->Instance) < (uint32_t)(DMA2_Channel1))
	{
		/* DMA1 */
		pDMAHandle->ChannelIndex = (((uint32_t)pDMAHandle->Instance - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2;
		pDMAHandle->DmaBaseAddress = DMA1;
	}
	else
	{
		/* DMA2 */
		pDMAHandle->ChannelIndex = (((uint32_t)pDMAHandle->Instance - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2;
		pDMAHandle->DmaBaseAddress = DMA2;
	}

	// 2. Change DMA peripheral state
	pDMAHandle->State = DMA_STATE_BUSY;

	// 3. Get the CR register value
	config = pDMAHandle->Instance->CCR;

	// 4. Clear PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits
	config &= ((uint32_t)~(DMA_CCR_PL    | DMA_CCR_MSIZE  | DMA_CCR_PSIZE  | \
	                      DMA_CCR_MINC  | DMA_CCR_PINC   | DMA_CCR_CIRC   | \
	                      DMA_CCR_DIR));

	// 5. Prepare the DMA Channel configuration
	config |=  pDMAHandle->Init.Direction		   |
				pDMAHandle->Init.PeriphInc           | pDMAHandle->Init.MemInc           |
				pDMAHandle->Init.PeriphDataAlignment | pDMAHandle->Init.MemDataAlignment |
				pDMAHandle->Init.Mode                | pDMAHandle->Init.Priority;

	// 6. Write to DMA Channel CR register
	pDMAHandle->Instance->CCR = config;

	// 7. Initialize the DMA state
	pDMAHandle->State = DMA_STATE_READY;
}


static void DMA_SetConfig(DMA_HandleTypeDef *pDMAHandle, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
	// 1. Clear all flags
	pDMAHandle->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << pDMAHandle->ChannelIndex);

	// 2. Configure DMA Channel data length
	pDMAHandle->Instance->CNDTR = DataLength;

	// 3a. Case of Memory to Peripheral
	if((pDMAHandle->Init.Direction) == DMA_MEMORY_TO_PERIPH)
	{
		// Configure DMA Channel destination address
		pDMAHandle->Instance->CPAR = DstAddress;

		// Configure DMA Channel source address
		pDMAHandle->Instance->CMAR = SrcAddress;
	}
	else // 3b. Case of Peripheral to Memory
	{
		// Configure DMA Channel source address
		pDMAHandle->Instance->CPAR = SrcAddress;

		// Configure DMA Channel destination address
		pDMAHandle->Instance->CMAR = DstAddress;
	}
}


void DMA_Start_IT(DMA_HandleTypeDef *pDMAHandle, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
	// BUSY에서 READY로 안바뀌어서 생기는 문제였다. READY인지 체크하는 조건문을 제거하면 잘됨

	// 1. Disable the peripheral
	DISABLE_DMA(pDMAHandle);

	// 2. Configure the source, destination address and the data length & clear flags
	DMA_SetConfig(pDMAHandle, SrcAddress, DstAddress, DataLength);

	// 3. Enable the transfer complete interrupt and the transfer error interrupt
	DISABLE_DMA_IT(pDMAHandle, DMA_IT_HT);
	ENABLE_DMA_IT(pDMAHandle, (DMA_IT_TC));

	// 4. Enable the peripheral
	ENABLE_DMA(pDMAHandle);
}


void DMA_IRQ_Handling(DMA_HandleTypeDef *pDMAHandle)
{
	/* Interrupt handling for DMA */

	uint32_t temp1, temp2;

	// 1. Handle for interrupt generated by Half transfer complete
	temp1 = (pDMAHandle->DmaBaseAddress->ISR & (DMA_FLAG_HT1 << pDMAHandle->ChannelIndex));
	temp2 = (pDMAHandle->Instance->CCR & DMA_IT_HT);

	if((temp1 != RESET) && (temp2 != RESET))
	{
		// 1. Clear the half transfer complete flag
		DMA_CLEAR_FLAG(pDMAHandle, DMA_GET_HT_FLAG_INDEX(pDMAHandle));

		// 2. Call Half transfer complete Callback
		pDMAHandle->XferHalfCpltCallback(pDMAHandle);
	}

	// 2. Handle for interrupt generated by Full transfer complete
	temp1 = (pDMAHandle->DmaBaseAddress->ISR & (DMA_FLAG_TC1 << pDMAHandle->ChannelIndex));
	temp2 = (pDMAHandle->Instance->CCR & DMA_IT_TC);

	if((temp1 != RESET) && (temp2 != RESET))
	{
		// 1. Change the DMA state
		pDMAHandle->State = DMA_STATE_READY;

		// 2. Clear the transfer complete flag
		DMA_CLEAR_FLAG(pDMAHandle, DMA_GET_TC_FLAG_INDEX(pDMAHandle));

		// 3. Call Full transfer complete Callback
		pDMAHandle->XferCpltCallback(pDMAHandle);
	}

	// 3. Handle for interrupt generated by Transfer error
	temp1 = (pDMAHandle->DmaBaseAddress->ISR & (DMA_FLAG_TE1 << pDMAHandle->ChannelIndex));
	temp2 = (pDMAHandle->Instance->CCR & DMA_IT_TE);

	if((temp1 != RESET) && (temp2 != RESET))
	{
		// 1. Clear all flags
		DMA_CLEAR_FLAG(pDMAHandle, DMA_GET_GI_FLAG_INDEX(pDMAHandle));

		// 2. Change the DMA state
		pDMAHandle->State = DMA_STATE_READY;

		// 3. Call Transfer error Callback
		pDMAHandle->XferErrorCallback(pDMAHandle);
	}
}

