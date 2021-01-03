/*
 * bldc.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Ganghyeok Lim
 */


#include "bldc.h"


uint8_t DutyRef = 0;

/****************************************************************************************/
/*																						*/
/*									BLDC Motor functions								*/
/*																						*/
/****************************************************************************************/

void BLDC_Drive(BLDC_HandleTypeDef *pBLDCHandle, uint16_t hallPhase)
{
	switch (hallPhase)
	{
		case Phase1:
		{
			BLDC_Step2(pBLDCHandle);
			break;
		}

		case Phase2:
		{
			BLDC_Step1(pBLDCHandle);
			break;
		}

		case Phase3:
		{
			BLDC_Step6(pBLDCHandle);
			break;
		}

		case Phase4:
		{
			BLDC_Step5(pBLDCHandle);
			break;
		}

		case Phase5:
		{
			BLDC_Step4(pBLDCHandle);
			break;
		}

		case Phase6:
		{
			BLDC_Step3(pBLDCHandle);
			break;
		}

		default :
			break;
	}
}


void BLDC_BootstrapCap_Charge(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. Clear GPIO pin of Top side(UT, VT, WT) and Disable All PWM channels
	GPIO_WritePin(pBLDCHandle->GPIO_List.GPIOx_Top, pBLDCHandle->GPIO_List.GPIO_Pins_Top, GPIO_PIN_RESET);
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_2);
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_3);
	Delay_ms(10);

	// 2. Re-initialize GPIO pins from TIM PWM channels to GPIO Output mode
	GPIO_InitTypeDef GPIOInit;

	memset(&GPIOInit, 0, sizeof(GPIOInit));

	// 3. Re-initialize GPIO pins to GPIO Output mode
	GPIOInit.Pin = pBLDCHandle->GPIO_List.GPIO_Pins_Bottom;
	GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOInit.Pull = GPIO_NOPULL;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(pBLDCHandle->GPIO_List.GPIOx_Bottom, &GPIOInit);
	Delay_ms(10);

	// 4. Charge Bootstrap Capacitor for 10ms
	GPIO_WritePin(pBLDCHandle->GPIO_List.GPIOx_Bottom, pBLDCHandle->GPIO_List.GPIO_Pins_Bottom, GPIO_PIN_SET);
	Delay_ms(10);
	GPIO_WritePin(pBLDCHandle->GPIO_List.GPIOx_Bottom, pBLDCHandle->GPIO_List.GPIO_Pins_Bottom, GPIO_PIN_RESET);

	// 5. Re-initialize GPIO pins from GPIO Output mode to TIM PWM channels
	memset(&GPIOInit, 0, sizeof(GPIOInit));

	GPIOInit.Pin = pBLDCHandle->GPIO_List.GPIO_Pins_Bottom;
	GPIOInit.Mode = GPIO_MODE_AF_PP;
	GPIOInit.Pull = GPIO_NOPULL;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(pBLDCHandle->GPIO_List.GPIOx_Bottom, &GPIOInit);
	Delay_ms(10);

	// 6. Enable All PWM channels
	TIM_ENABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_1);
	TIM_ENABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_2);
	TIM_ENABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_3);
	Delay_ms(10);
}


void BLDC_Step1(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. UT Logic On (PB0)
	GPIO_ModifyPin(GPIOB, pBLDCHandle->GPIO_List.GPIO_Pin_UT, (pBLDCHandle->GPIO_List.GPIO_Pin_VT | pBLDCHandle->GPIO_List.GPIO_Pin_WT));

	// 2. VB PWM On (PB7)
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_3);
	TIM_ENABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_2);
}

void BLDC_Step2(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. WT Logic On (PB2)
	GPIO_ModifyPin(GPIOB, pBLDCHandle->GPIO_List.GPIO_Pin_WT, (pBLDCHandle->GPIO_List.GPIO_Pin_UT | pBLDCHandle->GPIO_List.GPIO_Pin_VT));

	// 2. VB PWM On (PB7)
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_3);
	TIM_ENABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_2);
}

void BLDC_Step3(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. WT Logic On (PB2)
	GPIO_ModifyPin(GPIOB, pBLDCHandle->GPIO_List.GPIO_Pin_WT, (pBLDCHandle->GPIO_List.GPIO_Pin_UT | pBLDCHandle->GPIO_List.GPIO_Pin_VT));

	// 2. UB PWM On (PB6)
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_2);
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_3);
	TIM_ENABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_1);
}

void BLDC_Step4(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. VT Logic On (PB1)
	GPIO_ModifyPin(GPIOB, pBLDCHandle->GPIO_List.GPIO_Pin_VT, (pBLDCHandle->GPIO_List.GPIO_Pin_UT | pBLDCHandle->GPIO_List.GPIO_Pin_WT));

	// 2. UB PWM On (PB6)
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_2);
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_3);
	TIM_ENABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_1);
}

void BLDC_Step5(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. VT Logic On (PB1)
	GPIO_ModifyPin(GPIOB, pBLDCHandle->GPIO_List.GPIO_Pin_VT, (pBLDCHandle->GPIO_List.GPIO_Pin_UT | pBLDCHandle->GPIO_List.GPIO_Pin_WT));

	// 2. WB PWM On (PB8)
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_2);
	TIM_ENABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_3);
}

void BLDC_Step6(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. UT Logic On (PB0)
	GPIO_ModifyPin(GPIOB, pBLDCHandle->GPIO_List.GPIO_Pin_UT, (pBLDCHandle->GPIO_List.GPIO_Pin_VT | pBLDCHandle->GPIO_List.GPIO_Pin_WT));

	// 2. WB PWM On (PB8)
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_2);
	TIM_ENABLE_CHANNEL(pBLDCHandle->TIM_Handle, TIM_CHANNEL_3);
}

/****************************************************************************************/
/*																						*/
/*									 Test Functions										*/
/*																						*/
/****************************************************************************************/

void IR2101_Test1(uint16_t Top_time_us, uint16_t Low_time_us, uint16_t Dead_time_us)
{
	GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	Delay_us(Low_time_us);
	GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	Delay_us(Dead_time_us);
	GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	Delay_us(Top_time_us);
	GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	Delay_us(Dead_time_us);
}

void IR2101_Test2(uint16_t Top_time_ms, uint16_t Low_time_ms, uint16_t Dead_time_ms)
{
	GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	Delay_ms(Low_time_ms);
	GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	Delay_ms(Dead_time_ms);
	GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	Delay_ms(Top_time_ms);
	GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	Delay_ms(Dead_time_ms);
}

void IR2101_Test3(TIM_HandleTypeDef *pTIMHandle, uint16_t Top_time_us, uint16_t Low_time_us, uint16_t Dead_time_us)
{
	TIM_SET_COMPARE(pTIMHandle, TIM_CHANNEL_1, 50);
	Delay_us(Low_time_us);
	TIM_SET_COMPARE(pTIMHandle, TIM_CHANNEL_1, 0);
	Delay_us(Dead_time_us);
	GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	Delay_us(Top_time_us);
	GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	Delay_us(Dead_time_us);
}

void BootCharge_Test(uint16_t tim_on_time_us, uint16_t tim_off_time_us)
{
	GPIO_HandleTypeDef GPIOHandle_TIM1;
	memset(&GPIOHandle_TIM1, 0, sizeof(GPIOHandle_TIM1));

	// 1. Enable TIM1 & Config port for TIM1
	GPIOHandle_TIM1.Instance = GPIOA;
	GPIOHandle_TIM1.Init.Pin = GPIO_PIN_8;
	GPIOHandle_TIM1.Init.Mode = GPIO_MODE_AF_PP;
	GPIOHandle_TIM1.Init.Pull = GPIO_NOPULL;
	GPIOHandle_TIM1.Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(GPIOHandle_TIM1.Instance, &GPIOHandle_TIM1.Init);
	SET_BIT(TIM1->CR1, TIM_CR1_CEN);
	SET_BIT(TIM1->CCER, TIM_CCER_CC1E);


	Delay_us(tim_on_time_us);

	// 2. Disable TIM1 & Config port for GPIO Output mode
	CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);
	CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1E);
	GPIOHandle_TIM1.Instance = GPIOA;
	GPIOHandle_TIM1.Init.Pin = GPIO_PIN_8;
	GPIOHandle_TIM1.Init.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOHandle_TIM1.Init.Pull = GPIO_NOPULL;
	GPIOHandle_TIM1.Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(GPIOHandle_TIM1.Instance, &GPIOHandle_TIM1.Init);
	GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	Delay_us(tim_off_time_us);
}
