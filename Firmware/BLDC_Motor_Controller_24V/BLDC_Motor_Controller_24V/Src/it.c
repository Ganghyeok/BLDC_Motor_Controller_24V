/*
 * it.c
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#include "main.h"


void USART1_IRQHandler(void)
{
	USART_IRQHandling(&USART1Handle);
}
