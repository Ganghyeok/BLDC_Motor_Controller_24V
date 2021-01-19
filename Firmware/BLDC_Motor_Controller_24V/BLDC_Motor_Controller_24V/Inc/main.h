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
#include "tft.h"
#include "ts.h"


/* Application Specific Macro functions */
#define WAIT_BTN_PRESS(GPIOx, GPIO_PIN_NO)		WAIT_FLAG_CLEAR(GPIOx->IDR, GPIO_PIN_NO)
#define ENABLE_HALLSENSOR_EXTI()				NVIC_IRQConfig(IRQ_NO_EXTI9_5, NVIC_PRIOR_8, ENABLE)
#define DISABLE_HALLSENSOR_EXTI()				NVIC_IRQConfig(IRQ_NO_EXTI9_5, NVIC_PRIOR_8, DISABLE)

// DMA Macro function
#define ENABLE_DMA1_CHANNEL2()					SET_BIT(DMA1_Channel2->CCR, (0x1 << 0))
#define DISABLE_DMA1_CHANNEL2()					CLEAR_BIT(DMA1_Channel2->CCR, (0x1 << 0))
#define IS_IT_HT()								(DMA1->ISR & (0x1 << 26))
#define IS_IT_TC()								(DMA1->ISR & (0x1 << 25))
#define IS_IT_TE()								(DMA1->ISR & (0x1 << 27))

/* Extern Global variables */
extern TIM_HandleTypeDef TIM6Handle;
extern TIM_HandleTypeDef TIM4Handle;
extern BLDC_HandleTypeDef BLDC1Handle;
extern UART_HandleTypeDef UART3Handle;
extern DMA_HandleTypeDef DMA1Handle;
extern TFT_HandleTypeDef TFT1Handle;
extern TS_HandleTypeDef	TS1Handle;
extern SPI_HandleTypeDef SPI2Handle;
extern uint8_t ButtonFlag;
extern char MotorSpeedStr[6];
extern char MotorPositionStr[8];
extern char PwmPidStr[5];
extern char PwmPidAbsStr[4];
extern char Msg1[50];
extern uint8_t startFlag;


/* Extern Initialization functions */
extern void Button_Init(void);
extern void BLDC1_Init(void);
extern void UART3_Init(void);
extern void TIM6_Init(void);
extern void DMA1_Init(void);
extern void DMA1_Interrupt_Configuration(void);
extern void TFT1_Init(void);
extern void TS1_Init(void);
extern void Test_Init(void);


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
