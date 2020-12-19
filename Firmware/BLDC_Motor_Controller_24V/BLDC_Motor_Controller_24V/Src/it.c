/*
 * it.c
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#include "main.h"


void USART1_IRQHandler(void)
{

}


void TIM6_IRQHandler(void)
{
	TIM_IRQHandling(&TIM6Handle);
}
