/*
 * main.h
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f103xx.h"


#define WAIT_BTN_PRESS(GPIOx, GPIO_PIN_NO)		WAIT_FLAG_CLEAR(GPIOx->IDR, GPIO_PIN_NO)


extern TIM_HandleTypeDef TIM6Handle;
extern TIM_HandleTypeDef TIM1Handle;


extern void NVIC_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En_or_Di);
extern void SystemClock_Config(uint8_t clockFreq);
extern void Delay_us(uint32_t time_us);
extern void Delay_ms(uint32_t time_ms);
extern void GPIOTest_Init(void);
extern void UART1_Init(UART_HandleTypeDef *pUARTHandle);
extern void TIM6_Init(TIM_HandleTypeDef *pTIMHandle);
extern void TIM1_Init(TIM_HandleTypeDef *pTIMHandle);
extern void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pTIMHandle);

#endif /* MAIN_H_ */
