/*
 * bldc.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Ganghyeok Lim
 */


#include "main.h"


/****************************************************************************************/
/*																						*/
/*									BLDC Motor APIs										*/
/*																						*/
/****************************************************************************************/
void BLDC_BootstrapCap_Charge(void)
{
/* Disable All PWM channels */
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_2);
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_3);

/* Re-initialize GPIO pins from TIM PWM channels to GPIO Output mode */

	GPIO_InitTypeDef GPIOInit;
	memset(&GPIOInit, 0, sizeof(GPIOInit));

	// 1. Re-initialize GPIO pins to GPIO Output mode
	GPIOInit.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
	GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOInit.Pull = GPIO_NOPULL;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(GPIOA, &GPIOInit);

/* Charge Bootstrap Capacitor for 1ms */
	GPIO_WritePin(GPIOB, (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15), GPIO_PIN_RESET);
	GPIO_WritePin(GPIOA, (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10), GPIO_PIN_SET);
	Delay_ms(1);
	GPIO_WritePin(GPIOA, (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10), GPIO_PIN_RESET);

/* Re-initialize GPIO pins from GPIO Output mode to TIM PWM channels */
	memset(&GPIOInit, 0, sizeof(GPIOInit));
	GPIOInit.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
	GPIOInit.Mode = GPIO_MODE_AF_PP;
	GPIOInit.Pull = GPIO_NOPULL;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(GPIOA, &GPIOInit);

/* Enable All PWM channels */
	TIM_ENABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_1);
	TIM_ENABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_2);
	TIM_ENABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_3);
}


void BLDC_Step1(void)
{
	// 1. UT Logic On (PB13)
	GPIO_ModifyPin(GPIOB, GPIO_PIN_13, GPIO_PIN_14 | GPIO_PIN_15);

	// 2. VB PWM On (PA9)
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_3);
	TIM_ENABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_2);
}

void BLDC_Step2(void)
{
	// 1. WT Logic On (PB15)
	GPIO_ModifyPin(GPIOB, GPIO_PIN_15, GPIO_PIN_13 | GPIO_PIN_14);

	// 2. VB PWM On (PA9)
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_3);
	TIM_ENABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_2);
}

void BLDC_Step3(void)
{
	// 1. WT Logic On (PB15)
	GPIO_ModifyPin(GPIOB, GPIO_PIN_15, GPIO_PIN_13 | GPIO_PIN_14);

	// 2. UB PWM On (PA8)
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_2);
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_3);
	TIM_ENABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_1);
}

void BLDC_Step4(void)
{
	// 1. VT Logic On (PB14)
	GPIO_ModifyPin(GPIOB, GPIO_PIN_14, GPIO_PIN_13 | GPIO_PIN_15);

	// 2. UB PWM On (PA8)
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_2);
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_3);
	TIM_ENABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_1);
}

void BLDC_Step5(void)
{
	// 1. VT Logic On (PB14)
	GPIO_ModifyPin(GPIOB, GPIO_PIN_14, GPIO_PIN_13 | GPIO_PIN_15);

	// 2. WB PWM On (PA10)
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_2);
	TIM_ENABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_3);
}

void BLDC_Step6(void)
{
	// 1. UT Logic On (PB13)
	GPIO_ModifyPin(GPIOB, GPIO_PIN_13, GPIO_PIN_14 | GPIO_PIN_15);

	// 2. WB PWM On (PA10)
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_2);
	TIM_ENABLE_CHANNEL(&TIM1Handle, TIM_CHANNEL_3);
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

void IR2101_Test3(uint16_t Top_time_us, uint16_t Low_time_us, uint16_t Dead_time_us)
{
	TIM_SET_COMPARE(&TIM1Handle, TIM_CHANNEL_1, 50);
	Delay_us(Low_time_us);
	TIM_SET_COMPARE(&TIM1Handle, TIM_CHANNEL_1, 0);
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
