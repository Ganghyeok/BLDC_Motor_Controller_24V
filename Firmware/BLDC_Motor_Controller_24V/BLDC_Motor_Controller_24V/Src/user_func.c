/*
 * user_func.c
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#include "main.h"


TIM_HandleTypeDef TIM6Handle;
TIM_HandleTypeDef TIM4Handle;
BLDC_HandleTypeDef BLDCHandle;

uint8_t MotorState = STOP;
uint8_t ButtonFlag = FLAG_RESET;
uint16_t HallPhase = 0;


/********************************************************************************************************************
 * 																											  		*
 *											Application Specific Function											*
 * 																											  		*
 ********************************************************************************************************************/

/********************************************************************************************************************
 *												Initialization Function												*
 ********************************************************************************************************************/

void Button_Init(void)
{
	GPIO_InitTypeDef GPIOInit;

	memset(&GPIOInit, 0, sizeof(GPIOInit));

	// 1. Initialize GPIO for START/STOP Button
	GPIOInit.Pin = GPIO_PIN_7;
	GPIOInit.Mode = GPIO_MODE_INPUT;
	GPIOInit.Pull = GPIO_PULLUP;
	GPIO_Init(GPIOA, &GPIOInit);
}


void BLDC_Init(BLDC_HandleTypeDef *pBLDCHandle)
{
	pBLDCHandle->GPIO_List.GPIOx_Top = GPIOB;
	pBLDCHandle->GPIO_List.GPIO_Pin_UT = GPIO_PIN_0;
	pBLDCHandle->GPIO_List.GPIO_Pin_VT = GPIO_PIN_1;
	pBLDCHandle->GPIO_List.GPIO_Pin_WT = GPIO_PIN_2;
	pBLDCHandle->GPIO_List.GPIO_Pins_Top = pBLDCHandle->GPIO_List.GPIO_Pin_UT | pBLDCHandle->GPIO_List.GPIO_Pin_VT | pBLDCHandle->GPIO_List.GPIO_Pin_WT;

	pBLDCHandle->GPIO_List.GPIOx_Bottom = GPIOB;
	pBLDCHandle->GPIO_List.GPIO_Pin_UB = GPIO_PIN_6;
	pBLDCHandle->GPIO_List.GPIO_Pin_VB = GPIO_PIN_7;
	pBLDCHandle->GPIO_List.GPIO_Pin_WB = GPIO_PIN_8;
	pBLDCHandle->GPIO_List.GPIO_Pins_Bottom = pBLDCHandle->GPIO_List.GPIO_Pin_UB | pBLDCHandle->GPIO_List.GPIO_Pin_VB | pBLDCHandle->GPIO_List.GPIO_Pin_WB;

	pBLDCHandle->GPIO_List.GPIOx_Hall = GPIOC;
	pBLDCHandle->GPIO_List.GPIO_Pin_HA = GPIO_PIN_6;
	pBLDCHandle->GPIO_List.GPIO_Pin_HB = GPIO_PIN_7;
	pBLDCHandle->GPIO_List.GPIO_Pin_HC = GPIO_PIN_8;
	pBLDCHandle->GPIO_List.GPIO_Pins_Hall = pBLDCHandle->GPIO_List.GPIO_Pin_HA | pBLDCHandle->GPIO_List.GPIO_Pin_HB | pBLDCHandle->GPIO_List.GPIO_Pin_HC;

	pBLDCHandle->TIM_Handle = &TIM4Handle;
	pBLDCHandle->TIM_Handle->Instance = TIM4;


	GPIO_BLDC_Init(pBLDCHandle);	// GPIO init for U/V/W and Charge Bootstrap Cap
	EXTI_Init(pBLDCHandle);			// EXTI init for Interrupt triggered by Hallphase change
	TIM4_Init(pBLDCHandle);			// TIM1 init for PWM generation to drive BLDC motor
}


void GPIO_BLDC_Init(BLDC_HandleTypeDef *pBLDCHandle)
{
	GPIO_InitTypeDef GPIOInit;

	memset(&GPIOInit, 0, sizeof(GPIOInit));

	// 1. Initialize GPIO for UT, VT, WT to GPIO Output mode
	GPIOInit.Pin = pBLDCHandle->GPIO_List.GPIO_Pins_Top;
	GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOInit.Pull = GPIO_NOPULL;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(pBLDCHandle->GPIO_List.GPIOx_Top, &GPIOInit);
	Delay_ms(10);

	GPIO_WritePin(pBLDCHandle->GPIO_List.GPIOx_Top, pBLDCHandle->GPIO_List.GPIO_Pins_Top, GPIO_PIN_RESET);


	// 2. Initialize GPIO for UB, VB, WB to GPIO Output mode
	GPIOInit.Pin = pBLDCHandle->GPIO_List.GPIO_Pins_Bottom;
	GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOInit.Pull = GPIO_NOPULL;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(pBLDCHandle->GPIO_List.GPIOx_Bottom, &GPIOInit);
	Delay_ms(10);

	GPIO_WritePin(pBLDCHandle->GPIO_List.GPIOx_Bottom, pBLDCHandle->GPIO_List.GPIO_Pins_Bottom, GPIO_PIN_RESET);


	// 3. Charge Bootstrap Capacitor for 10ms
	Delay_ms(10);
	GPIO_WritePin(pBLDCHandle->GPIO_List.GPIOx_Bottom, pBLDCHandle->GPIO_List.GPIO_Pins_Bottom, GPIO_PIN_SET);
	Delay_ms(10);
	GPIO_WritePin(pBLDCHandle->GPIO_List.GPIOx_Bottom, pBLDCHandle->GPIO_List.GPIO_Pins_Bottom, GPIO_PIN_RESET);
}


void UART2_Init(UART_HandleTypeDef *pUARTHandle)
{
	pUARTHandle->Instance = USART2;
	pUARTHandle->Init.Mode = UART_MODE_TX;
	pUARTHandle->Init.OverSampling = UART_OVERSAMPLING_16;
	pUARTHandle->Init.BaudRate = USART_STD_BAUD_115200;
	pUARTHandle->Init.Parity = UART_PARITY_NONE;
	pUARTHandle->Init.StopBits = UART_STOPBITS_1;
	pUARTHandle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	pUARTHandle->Init.WordLength = UART_WORDLENGTH_8B;

	USART_Init(pUARTHandle);
}


void TIM6_Init(TIM_HandleTypeDef *pTIMHandle)
{
	// Init TIM6 Base
	pTIMHandle->Instance = TIM6;
	pTIMHandle->Init.CounterMode = TIM_COUNTERMODE_UP;
	pTIMHandle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pTIMHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	pTIMHandle->Init.Prescaler = (7200-1);	// 72MHz / 7200 = 10kHz
	pTIMHandle->Init.Period = (10-1);	// 10kHz / 10 = 1kHz
	pTIMHandle->Init.RepetitionCounter = 0;
	TIM_Base_Init(pTIMHandle);

	// Enable TIM6 interrupt for Update Event
	TIM_ENABLE_IT(&TIM6Handle, TIM_IT_UPDATE);

	// Enable TIM6 Counter
	TIM_ENABLE_COUNTER(&TIM6Handle);
}


void TIM1_Init(TIM_HandleTypeDef *pTIMHandle)
{
	pTIMHandle->Instance = TIM1;
	pTIMHandle->Init.CounterMode = TIM_COUNTERMODE_UP;
	pTIMHandle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pTIMHandle->Init.Prescaler = (36-1);	//   72MHz / 36 = 2MHz
	pTIMHandle->Init.Period = (100-1);		//   2MHz / 100 = 20kHz
	TIM_PWM_Init(pTIMHandle);

	TIM_OC_InitTypeDef TIM1_PWMConfig;

	memset(&TIM1_PWMConfig, 0, sizeof(TIM1_PWMConfig));

	TIM1_PWMConfig.OCMode = TIM_OCMODE_PWM1;
	TIM1_PWMConfig.OCPolarity = TIM_OCPOLARITY_HIGH;

	TIM1_PWMConfig.Pulse = 0;	// Initially, 0% duty
	TIM_PWM_ConfigChannel(pTIMHandle, &TIM1_PWMConfig, TIM_CHANNEL_1);

	TIM1_PWMConfig.Pulse = 0;	// Initially, 0% duty
	TIM_PWM_ConfigChannel(pTIMHandle, &TIM1_PWMConfig, TIM_CHANNEL_2);

	TIM1_PWMConfig.Pulse = 0;	// Initially, 0% duty
	TIM_PWM_ConfigChannel(pTIMHandle, &TIM1_PWMConfig, TIM_CHANNEL_3);
}


void TIM3_Init(TIM_HandleTypeDef *pTIMHandle)
{
	pTIMHandle->Instance = TIM3;
	pTIMHandle->Init.CounterMode = TIM_COUNTERMODE_UP;
	pTIMHandle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pTIMHandle->Init.Prescaler = (36-1);	//   72MHz / 36 = 2MHz
	pTIMHandle->Init.Period = (100-1);		//   2MHz / 100 = 20kHz
	TIM_PWM_Init(pTIMHandle);

	TIM_OC_InitTypeDef TIM3_PWMConfig;

	memset(&TIM3_PWMConfig, 0, sizeof(TIM3_PWMConfig));

	TIM3_PWMConfig.OCMode = TIM_OCMODE_PWM1;
	TIM3_PWMConfig.OCPolarity = TIM_OCPOLARITY_HIGH;

	TIM3_PWMConfig.Pulse = 0;	// Initially, 0% duty
	TIM_PWM_ConfigChannel(pTIMHandle, &TIM3_PWMConfig, TIM_CHANNEL_1);

	TIM3_PWMConfig.Pulse = 0;	// Initially, 0% duty
	TIM_PWM_ConfigChannel(pTIMHandle, &TIM3_PWMConfig, TIM_CHANNEL_2);

	TIM3_PWMConfig.Pulse = 0;	// Initially, 0% duty
	TIM_PWM_ConfigChannel(pTIMHandle, &TIM3_PWMConfig, TIM_CHANNEL_3);
}


void TIM4_Init(BLDC_HandleTypeDef *pBLDCHandle)
{
	pBLDCHandle->TIM_Handle->Init.CounterMode = TIM_COUNTERMODE_UP;
	pBLDCHandle->TIM_Handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pBLDCHandle->TIM_Handle->Init.Prescaler = (36-1);		//   72MHz / 36 = 2MHz
	pBLDCHandle->TIM_Handle->Init.Period = (100-1);		//   2MHz / 100 = 20kHz
	TIM_PWM_Init(pBLDCHandle->TIM_Handle);

	TIM_OC_InitTypeDef TIM4_PWMConfig;

	memset(&TIM4_PWMConfig, 0, sizeof(TIM4_PWMConfig));

	TIM4_PWMConfig.OCMode = TIM_OCMODE_PWM1;
	TIM4_PWMConfig.OCPolarity = TIM_OCPOLARITY_HIGH;

	TIM4_PWMConfig.Pulse = 0;	// Initially, 0% duty
	TIM_PWM_ConfigChannel(pBLDCHandle->TIM_Handle, &TIM4_PWMConfig, TIM_CHANNEL_1);

	TIM4_PWMConfig.Pulse = 0;	// Initially, 0% duty
	TIM_PWM_ConfigChannel(pBLDCHandle->TIM_Handle, &TIM4_PWMConfig, TIM_CHANNEL_2);

	TIM4_PWMConfig.Pulse = 0;	// Initially, 0% duty
	TIM_PWM_ConfigChannel(pBLDCHandle->TIM_Handle, &TIM4_PWMConfig, TIM_CHANNEL_3);
}


void EXTI_Init(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. Configure GPIO of EXTI
	GPIO_InitTypeDef GPIOInit;

	memset(&GPIOInit, 0, sizeof(GPIOInit));

	GPIOInit.Pin = pBLDCHandle->GPIO_List.GPIO_Pins_Hall;
	GPIOInit.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIOInit.Pull = GPIO_NOPULL;
	GPIO_Init(pBLDCHandle->GPIO_List.GPIOx_Hall, &GPIOInit);

	// 2. Configure NVIC of EXTI
	//NVIC_IRQConfig(IRQ_NO_EXTI9_5, NVIC_PRIOR_8, ENABLE);		// NVIC IRQ for EXTI will be enabled in main function when START Button is pressed
}


/********************************************************************************************************************
 *												  Callback Function													*
 ********************************************************************************************************************/

void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pTIMHandle)
{
	if(pTIMHandle->Instance == TIM6)
	{
		if(ButtonFlag == FLAG_RESET)
		{
			uint8_t buttonState;

			// Check the Button is pressed
			buttonState = READ_BIT(GPIOA->IDR, GPIO_PIN_7);

			if(buttonState == BUTTON_PRESSED)
			{
				ButtonFlag = FLAG_SET;
			}
		}
	}
}


void EXTI_Callback(uint32_t GPIO_Pin)
{
	// 1. Detect current HallPhase location
	HallPhase = (READ_BIT(GPIOC->IDR, BLDCHandle.GPIO_List.GPIO_Pins_Hall)) >> 6U;

	// 2. Drive BLDC motor according to HallPhase location
	BLDC_Drive(&BLDCHandle, HallPhase);


	UNUSED(GPIO_Pin);
}


/********************************************************************************************************************
 *							Group of functions which belong to main function for increasing Readability				*
 ********************************************************************************************************************/

void MemsetHandleStructure(void)
{
	memset(&TIM6Handle, 0, sizeof(TIM6Handle));
	memset(&TIM4Handle, 0, sizeof(TIM4Handle));
	memset(&BLDCHandle, 0, sizeof(BLDCHandle));
}


void StartTimerPwm(BLDC_HandleTypeDef *pBLDCHandle)
{
	TIM_PWM_Start(pBLDCHandle->TIM_Handle, TIM_CHANNEL_1);			// Start PWM for UB
	TIM_PWM_Start(pBLDCHandle->TIM_Handle, TIM_CHANNEL_2);			// Start PWM for VB
	TIM_PWM_Start(pBLDCHandle->TIM_Handle, TIM_CHANNEL_3);			// Start PWM for WB
}


void EnableTimerPwmChannel(BLDC_HandleTypeDef *pBLDCHandle)
{
	TIM_ENABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_1);
	TIM_ENABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_2);
	TIM_ENABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_3);
}


void DisableTimerPwmChannel(BLDC_HandleTypeDef *pBLDCHandle)
{
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_2);
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_3);
}


void SetPwmDuty(BLDC_HandleTypeDef *pBLDCHandle, uint32_t duty)
{
	TIM_SET_COMPARE(pBLDCHandle->TIM_Handle, TIM_CHANNEL_1, (uint16_t)duty);
	TIM_SET_COMPARE(pBLDCHandle->TIM_Handle, TIM_CHANNEL_2, (uint16_t)duty);
	TIM_SET_COMPARE(pBLDCHandle->TIM_Handle, TIM_CHANNEL_3, (uint16_t)duty);
}

