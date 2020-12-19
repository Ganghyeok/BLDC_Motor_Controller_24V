/*
 * msp.c
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#include "main.h"


void USART_MspInit(USART_TypeDef *USARTx)
{
	// 1. Configure GPIO for USART
	GPIO_HandleTypeDef GPIOHandle;

	memset(&GPIOHandle, 0, sizeof(GPIOHandle));

	if(USARTx == USART1)
	{
		// USART1 Tx
		GPIOHandle.Instance = GPIOA;
		GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		GPIOHandle.Init.Pin = GPIO_PIN_9;
		GPIOHandle.Init.Pull = GPIO_PULLUP;
		GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_HIGH;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);

		// USART1 Rx
		GPIOHandle.Init.Mode = GPIO_MODE_INPUT;
		GPIOHandle.Init.Pin = GPIO_PIN_10;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);
	}

	// 2. Configure CLOCK for USART
	USART_PeripheralClockControl(USARTx, ENABLE);
}



void TIM_Base_MspInit(TIM_TypeDef *TIMx)
{
//	// 1. Configure GPIO for TIM
//
//	GPIO_HandleTypeDef TIMx_GPIOHandle;
//
//	memset(&TIMx_GPIOHandle, 0, sizeof(TIMx_GPIOHandle));
//
//	if(TIMx == TIM1)
//	{
//		TIMx_GPIOHandle.Instance = GPIOA;
//		TIMx_GPIOHandle.Init.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
//		TIMx_GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
//		TIMx_GPIOHandle.Init.Pull = GPIO_NOPULL;
//		TIMx_GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
//		GPIO_Init(GPIOA, &TIMx_GPIOHandle.Init);
//	}
//
//	// 2. Configure CLOCK for TIM
	TIM_PeripheralClockControl(TIMx, ENABLE);
}
