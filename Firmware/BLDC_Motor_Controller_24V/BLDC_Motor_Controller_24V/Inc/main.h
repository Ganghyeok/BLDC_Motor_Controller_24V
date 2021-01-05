/*
 * main.h
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f103xx.h"
#include "common.h"
#include "bldc.h"


/* Application Specific Macro functions */
#define WAIT_BTN_PRESS(GPIOx, GPIO_PIN_NO)		WAIT_FLAG_CLEAR(GPIOx->IDR, GPIO_PIN_NO)
#define ENABLE_HALLSENSOR_EXTI()				NVIC_IRQConfig(IRQ_NO_EXTI9_5, NVIC_PRIOR_8, ENABLE)
#define DISABLE_HALLSENSOR_EXTI()				NVIC_IRQConfig(IRQ_NO_EXTI9_5, NVIC_PRIOR_8, DISABLE)


/* Extern Global variables */
extern TIM_HandleTypeDef TIM6Handle;
extern TIM_HandleTypeDef TIM4Handle;
extern BLDC_HandleTypeDef BLDC1Handle;
extern UART_HandleTypeDef UART2Handle;
extern uint8_t ButtonFlag;
extern uint8_t msg1[100];


/* Extern Initialization functions */
extern void Button_Init(void);
extern void BLDC1_Init(void);
extern void UART2_Init(void);
extern void TIM6_Init(void);


/* Extern Callback functions */
extern void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pTIMHandle);
extern void EXTI_Callback(uint32_t GPIO_Pin);


/* Extern Group of functions which belong to main function */
extern void MemsetHandleStructure(void);
extern void StartTimerPwm(BLDC_HandleTypeDef *pBLDCHandle);
extern void EnableTimerPwmChannel(BLDC_HandleTypeDef *pBLDCHandle);
extern void DisableTimerPwmChannel(BLDC_HandleTypeDef *pBLDCHandle);
extern void SetPwmDuty(BLDC_HandleTypeDef *pBLDCHandle, uint32_t duty);



#endif /* MAIN_H_ */
