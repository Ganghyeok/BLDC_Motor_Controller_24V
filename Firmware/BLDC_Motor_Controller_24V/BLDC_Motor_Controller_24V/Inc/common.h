/*
 * common.h
 *
 *  Created on: 2021. 1. 3.
 *      Author: Ganghyeok Lim
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "stm32f103xx.h"


/* User Common functions */
void NVIC_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En_or_Di);
void SystemClock_Config(uint8_t clockFreq);
void Delay_us(uint32_t time_us);
void Delay_ms(uint32_t time_ms);



#endif /* COMMON_H_ */
