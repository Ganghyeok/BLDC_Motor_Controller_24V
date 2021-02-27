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

/* Application Specific Macro */
#define STATE_MENU								0
#define STATE_SPEED								1
#define STATE_POSITION							2
#define STATE_POSITION_TRACKING					3
#define STATE_END								4

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
/* Peripheral Handle Definitions */
extern TIM_HandleTypeDef 						TIM6Handle;
extern TIM_HandleTypeDef 						TIM4Handle;
extern BLDC_HandleTypeDef 						BLDC1Handle;
extern UART_HandleTypeDef 						UART3Handle;
extern DMA_HandleTypeDef 						DMA1Handle;
extern TFT_HandleTypeDef 						TFT1Handle;
extern TS_HandleTypeDef							TS1Handle;
extern SPI_HandleTypeDef 						SPI2Handle;

/* Status Flags */
extern uint8_t 									State;
extern uint8_t 									State_option;
extern uint8_t 									Recharge_flag;

/* Key Variables */
extern int32_t 									Mode_key;				// Count of 'MODE' Key
extern int32_t 									Up_key;					// Count of 'UP' Key
extern int32_t									Down_key;				// Count of 'DOWN' Key
extern int32_t 									Start_key;				// Count of 'START/STOP' Key

extern uint8_t 									KeyFlag_Mode;
extern uint8_t 									KeyFlag_Up;
extern uint8_t 									KeyFlag_Down;
extern uint8_t 									KeyFlag_Start;

extern uint32_t 								KeyTime_Mode;
extern uint32_t 								KeyTime_Up;
extern uint32_t 								KeyTime_Down;
extern uint32_t 								KeyTime_Start;

/* Touch Screen Variables */
extern uint8_t 									TouchDetection_flag;
extern uint32_t 								TouchTime;
extern uint16_t 								xTouch_log;
extern uint16_t 								yTouch_log;

/* Strings for UART */
extern char 									MotorSpeedStr[6];
extern char 									MotorPositionStr[8];
extern char 									Msg1[50];

/* Graph variables */
extern uint16_t 								x;
extern uint16_t 								y;
extern uint16_t 								x_prv;
extern uint16_t 								y_prv;
extern uint8_t 									GraphDraw_flag;
extern uint8_t 									GraphClear_flag;

/* Extern Initialization functions */
extern void Key_Init(void);
extern void BLDC1_Init(void);
extern void UART3_Init(void);
extern void TIM6_Init(void);
extern void DMA1_Init(void);
extern void TFT1_Init(void);
extern void TS1_Init(void);
extern void Test_Init(void);


/* Extern Callback functions */
extern void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pTIMHandle);
extern void BLDC_SpeedMode(void);
extern void BLDC_PositionMode(void);
extern void Detect_KeyInput(void);
extern void Detect_TouchScreenInput(void);
extern void EXTI_Callback(uint32_t GPIO_Pin);


/* Extern Group of functions which belong to main function */
extern void State_Menu(void);
extern void State_Speed(void);
extern void State_Position(void);
extern void State_Position_Tracking(void);
extern void Reset_All_Variables(void);
extern void Reset_Speed_Variables(void);
extern void Reset_Position_Variables(void);
extern void Draw_axis(TFT_HandleTypeDef *pTFTHandle, uint8_t state);
extern void Draw_Graph(TFT_HandleTypeDef *pTFTHandle);
extern void Clear_Graph(TFT_HandleTypeDef *pTFTHandle);

extern void MemsetHandleStructure(void);
extern void StartTimerPwm(BLDC_HandleTypeDef *pBLDCHandle);
extern void EnableTimerPwmChannel(BLDC_HandleTypeDef *pBLDCHandle);
extern void DisableTimerPwmChannel(BLDC_HandleTypeDef *pBLDCHandle);
extern void SetPwmDuty(BLDC_HandleTypeDef *pBLDCHandle, uint32_t duty);



#endif /* MAIN_H_ */
