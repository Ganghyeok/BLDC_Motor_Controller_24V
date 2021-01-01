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


#define Phase1			3
#define Phase2			2
#define Phase3			6
#define Phase4			4
#define Phase5			5
#define Phase6			1


extern TIM_HandleTypeDef TIM6Handle;
extern TIM_HandleTypeDef TIM1Handle;

extern void NVIC_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En_or_Di);
extern void SystemClock_Config(uint8_t clockFreq);
extern void Delay_us(uint32_t time_us);
extern void Delay_ms(uint32_t time_ms);

extern void GPIO_BLDC_Init(void);
extern void UART2_Init(UART_HandleTypeDef *pUARTHandle);
extern void TIM6_Init(TIM_HandleTypeDef *pTIMHandle);
extern void TIM1_Init(TIM_HandleTypeDef *pTIMHandle);
extern void TIM3_Init(TIM_HandleTypeDef *pTIMHandle);
extern void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pTIMHandle);
extern void EXTI_Init(GPIO_HandleTypeDef *GPIOHandle);
extern void EXTI_Callback(uint32_t GPIO_Pin);

extern void BLDC_BootstrapCap_Charge(void);
extern void BLDC_Step1(void);
extern void BLDC_Step2(void);
extern void BLDC_Step3(void);
extern void BLDC_Step4(void);
extern void BLDC_Step5(void);
extern void BLDC_Step6(void);


extern void IR2101_Test1(uint16_t Top_time_us, uint16_t Low_time_us, uint16_t Dead_time_us);
extern void IR2101_Test2(uint16_t Top_time_ms, uint16_t Low_time_ms, uint16_t Dead_time_ms);
extern void IR2101_Test3(uint16_t Top_time_ms, uint16_t Low_time_ms, uint16_t Dead_time_ms);
extern void BootCharge_Test(uint16_t tim_on_time_us, uint16_t tim_off_time_us);

#endif /* MAIN_H_ */
