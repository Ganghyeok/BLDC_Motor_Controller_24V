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


void DMA1_Channel2_IRQHandler(void)
{
	DMA_IRQ_Handling(UART3Handle.hdmatx);
}
