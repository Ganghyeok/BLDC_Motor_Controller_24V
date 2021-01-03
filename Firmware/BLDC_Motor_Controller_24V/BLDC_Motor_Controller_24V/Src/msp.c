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
	else if(USARTx == USART2)
	{
		// USART2 Tx
		GPIOHandle.Instance = GPIOA;
		GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		GPIOHandle.Init.Pin = GPIO_PIN_2;
		GPIOHandle.Init.Pull = GPIO_PULLUP;
		GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_HIGH;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);

		// USART2 Rx
		GPIOHandle.Init.Mode = GPIO_MODE_INPUT;
		GPIOHandle.Init.Pin = GPIO_PIN_3;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);
	}

	// 2. Configure CLOCK for USART
	USART_PeripheralClockControl(USARTx, ENABLE);
}


void TIM_Base_MspInit(TIM_TypeDef *TIMx)
{
	if(TIMx == TIM6)
	{
		// 1. Configure GPIO for TIM
		// TIM6 is used for just time base generation so that GPIO config is not needed

		// 2. Configure CLOCK for TIM
		TIM_PeripheralClockControl(TIMx, ENABLE);

		// 3. Configure NVIC for TIM
		NVIC_IRQConfig(IRQ_NO_TIM6, NVIC_PRIOR_15, ENABLE);
	}

}


void TIM_PWM_MspInit(TIM_HandleTypeDef *pTIMHandle)
{
	// 1. Configure the GPIO for TIM
	GPIO_HandleTypeDef TIMx_GPIOHandle;

	if(pTIMHandle->Instance == TIM1)
	{
		memset(&TIMx_GPIOHandle, 0, sizeof(TIMx_GPIOHandle));

		TIMx_GPIOHandle.Instance = GPIOA;
		TIMx_GPIOHandle.Init.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
		TIMx_GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		TIMx_GPIOHandle.Init.Pull = GPIO_NOPULL;
		TIMx_GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_Init(TIMx_GPIOHandle.Instance, &TIMx_GPIOHandle.Init);
	}

	if(pTIMHandle->Instance == TIM3)
	{
		memset(&TIMx_GPIOHandle, 0, sizeof(TIMx_GPIOHandle));

		TIMx_GPIOHandle.Instance = GPIOC;
		TIMx_GPIOHandle.Init.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
		TIMx_GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		TIMx_GPIOHandle.Init.Pull = GPIO_NOPULL;
		TIMx_GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_Init(TIMx_GPIOHandle.Instance, &TIMx_GPIOHandle.Init);
	}

	if(pTIMHandle->Instance == TIM4)
	{
		memset(&TIMx_GPIOHandle, 0, sizeof(TIMx_GPIOHandle));

		TIMx_GPIOHandle.Instance = GPIOB;
		TIMx_GPIOHandle.Init.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
		TIMx_GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		TIMx_GPIOHandle.Init.Pull = GPIO_NOPULL;
		TIMx_GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_Init(TIMx_GPIOHandle.Instance, &TIMx_GPIOHandle.Init);
	}

	// 2. Configure CLOCK for TIM
	TIM_PeripheralClockControl(pTIMHandle->Instance, ENABLE);
}
