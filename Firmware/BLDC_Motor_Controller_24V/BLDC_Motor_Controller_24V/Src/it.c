/*
 * it.c
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#include "main.h"


void TIM6_IRQHandler(void)
{
	TIM_IRQHandling(&TIM6Handle);
}


void EXTI9_5_IRQHandler(void)
{
	EXTI_IRQHandling(BLDC1Handle.Init.GPIO_Pins_Hall);
}


void EXTI0_IRQHandler(void)
{
//	EXTI->PR |= GPIO_PIN_0;
//
//	GPIO_TogglePin(GPIOA, GPIO_PIN_1);
}


void DMA1_Channel7_IRQHandler(void)
{
	if( IS_IT_HT() )
	{
		DMA1->IFCR |= (1 << 26);
		DMA1_HT_Complete_Callback();
	}
	else if( IS_IT_TC() )
	{
		DMA1->IFCR |= (1 << 25);
		DMA1_FT_Complete_Callback();
	}
	else if( IS_IT_TE() )
	{
		DMA1->IFCR |= (1 << 27);
		DMA1_TE_Error_Callback();
	}
}
