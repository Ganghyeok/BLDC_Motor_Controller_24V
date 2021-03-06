/*
 * msp.c
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#include "main.h"


void USART_MspInit(UART_HandleTypeDef *pUARTHandle)
{
	// 1. Configure GPIO for USART
	GPIO_HandleTypeDef GPIOHandle;

	memset(&GPIOHandle, 0, sizeof(GPIOHandle));

	if(pUARTHandle->Instance == USART1)
	{
		// USART1 Tx
		GPIOHandle.Instance = GPIOA;
		GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		GPIOHandle.Init.Pin = GPIO_PIN_9;
		GPIOHandle.Init.Pull = GPIO_PULLUP;
		GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_HIGH;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);

		// USART1 Rx
		GPIOHandle.Init.Mode = GPIO_MODE_INPUT;
		GPIOHandle.Init.Pin = GPIO_PIN_10;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);
	}
	else if(pUARTHandle->Instance == USART2)
	{
		/* USART2 GPIO Configuration */

		// USART2 Tx
		GPIOHandle.Instance = GPIOA;
		GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		GPIOHandle.Init.Pin = GPIO_PIN_2;
		GPIOHandle.Init.Pull = GPIO_PULLUP;
		GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_HIGH;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);

		// USART2 Rx
		GPIOHandle.Init.Mode = GPIO_MODE_INPUT;
		GPIOHandle.Init.Pin = GPIO_PIN_3;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);

		/* USART2 DMA Configuration */
		pUARTHandle->hdmatx->Instance = DMA1_Channel7;
		pUARTHandle->hdmatx->Init.Direction = DMA_MEMORY_TO_PERIPH;
		pUARTHandle->hdmatx->Init.PeriphInc = DMA_PINC_DISABLE;
		pUARTHandle->hdmatx->Init.MemInc = DMA_MINC_ENABLE;
		pUARTHandle->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		pUARTHandle->hdmatx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		pUARTHandle->hdmatx->Init.Mode = DMA_NORMAL;
		pUARTHandle->hdmatx->Init.Priority = DMA_PRIORITY_LOW;
		DMA_Init(pUARTHandle->hdmatx);


		pUARTHandle->hdmatx = &DMA1Handle;
		DMA1Handle.Parent = pUARTHandle;
	}
	else if(pUARTHandle->Instance == USART3)
	{
		/* USART3 GPIO Configuration */

		// USART3 Tx
		GPIOHandle.Instance = GPIOB;
		GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		GPIOHandle.Init.Pin = GPIO_PIN_10;
		GPIOHandle.Init.Pull = GPIO_PULLUP;
		GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_HIGH;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);

		// USART3 Rx
		GPIOHandle.Init.Mode = GPIO_MODE_INPUT;
		GPIOHandle.Init.Pin = GPIO_PIN_11;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);

		/* USART3 DMA Configuration */
		pUARTHandle->hdmatx->Instance = DMA1_Channel2;
		pUARTHandle->hdmatx->Init.Direction = DMA_MEMORY_TO_PERIPH;
		pUARTHandle->hdmatx->Init.PeriphInc = DMA_PINC_DISABLE;
		pUARTHandle->hdmatx->Init.MemInc = DMA_MINC_ENABLE;
		pUARTHandle->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		pUARTHandle->hdmatx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		pUARTHandle->hdmatx->Init.Mode = DMA_NORMAL;
		pUARTHandle->hdmatx->Init.Priority = DMA_PRIORITY_LOW;
		DMA_Init(pUARTHandle->hdmatx);


		pUARTHandle->hdmatx = &DMA1Handle;
		DMA1Handle.Parent = pUARTHandle;
	}

	// 2. Configure CLOCK for USART
	USART_PeripheralClockControl(pUARTHandle->Instance, ENABLE);
}


void TIM_Base_MspInit(TIM_TypeDef *TIMx)
{
	if(TIMx == TIM6)
	{
		// 1. Configure GPIO for TIM
		// TIM6 is used for just time base generation so that GPIO config is not needed

		// 2. Configure CLOCK for TIM
		TIM_PeripheralClockControl(TIMx, ENABLE);

		// 3. Configure NVIC for TIM
		NVIC_IRQConfig(IRQ_NO_TIM6, NVIC_PRIOR_14, ENABLE);
	}
}


void TIM_PWM_MspInit(TIM_HandleTypeDef *pTIMHandle)
{
	// 1. Configure the GPIO for TIM
	GPIO_HandleTypeDef TIMx_GPIOHandle;

	if(pTIMHandle->Instance == TIM1)
	{
		memset(&TIMx_GPIOHandle, 0, sizeof(TIMx_GPIOHandle));

		TIMx_GPIOHandle.Instance = GPIOA;
		TIMx_GPIOHandle.Init.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
		TIMx_GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		TIMx_GPIOHandle.Init.Pull = GPIO_NOPULL;
		TIMx_GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_Init(TIMx_GPIOHandle.Instance, &TIMx_GPIOHandle.Init);
	}

	if(pTIMHandle->Instance == TIM3)
	{
		memset(&TIMx_GPIOHandle, 0, sizeof(TIMx_GPIOHandle));

		TIMx_GPIOHandle.Instance = GPIOC;
		TIMx_GPIOHandle.Init.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
		TIMx_GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		TIMx_GPIOHandle.Init.Pull = GPIO_NOPULL;
		TIMx_GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_Init(TIMx_GPIOHandle.Instance, &TIMx_GPIOHandle.Init);
	}

	if(pTIMHandle->Instance == TIM4)
	{
		memset(&TIMx_GPIOHandle, 0, sizeof(TIMx_GPIOHandle));

		TIMx_GPIOHandle.Instance = GPIOB;
		TIMx_GPIOHandle.Init.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
		TIMx_GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		TIMx_GPIOHandle.Init.Pull = GPIO_NOPULL;
		TIMx_GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_Init(TIMx_GPIOHandle.Instance, &TIMx_GPIOHandle.Init);
	}

	// 2. Configure CLOCK for TIM
	TIM_PeripheralClockControl(pTIMHandle->Instance, ENABLE);
}


void BLDC_MspInit(BLDC_HandleTypeDef *pBLDCHandle)
{
	GPIO_InitTypeDef GPIOInit;

	memset(&GPIOInit, 0, sizeof(GPIOInit));

	if(pBLDCHandle->Instance == BLDC1)
	{
		/************************************************************************
		 *			Low level init GPIO of UT/VT/WT, UB/VB/WB, HA/HB/HC			*
		 ***********************************************************************/
		BLDC1Handle.Init.GPIOx_Top = GPIOB;
		BLDC1Handle.Init.GPIO_Pin_UT = GPIO_PIN_0;
		BLDC1Handle.Init.GPIO_Pin_VT = GPIO_PIN_1;
		BLDC1Handle.Init.GPIO_Pin_WT = GPIO_PIN_2;
		BLDC1Handle.Init.GPIO_Pins_Top = BLDC1Handle.Init.GPIO_Pin_UT | BLDC1Handle.Init.GPIO_Pin_VT | BLDC1Handle.Init.GPIO_Pin_WT;

		BLDC1Handle.Init.GPIOx_Bottom = GPIOB;
		BLDC1Handle.Init.GPIO_Pin_UB = GPIO_PIN_6;
		BLDC1Handle.Init.GPIO_Pin_VB = GPIO_PIN_7;
		BLDC1Handle.Init.GPIO_Pin_WB = GPIO_PIN_8;
		BLDC1Handle.Init.GPIO_Pins_Bottom = BLDC1Handle.Init.GPIO_Pin_UB | BLDC1Handle.Init.GPIO_Pin_VB | BLDC1Handle.Init.GPIO_Pin_WB;

		BLDC1Handle.Init.GPIOx_Hall = GPIOA;
		BLDC1Handle.Init.GPIO_Pin_HA = GPIO_PIN_5;
		BLDC1Handle.Init.GPIO_Pin_HB = GPIO_PIN_6;
		BLDC1Handle.Init.GPIO_Pin_HC = GPIO_PIN_7;
		BLDC1Handle.Init.GPIO_Pins_Hall = BLDC1Handle.Init.GPIO_Pin_HA | BLDC1Handle.Init.GPIO_Pin_HB | BLDC1Handle.Init.GPIO_Pin_HC;


		// 1. Initialize GPIO for UT, VT, WT to GPIO Output mode
		GPIOInit.Pin = pBLDCHandle->Init.GPIO_Pins_Top;
		GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
		GPIOInit.Pull = GPIO_NOPULL;
		GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_Init(pBLDCHandle->Init.GPIOx_Top, &GPIOInit);
		Delay_ms(10);

		GPIO_WritePin(pBLDCHandle->Init.GPIOx_Top, pBLDCHandle->Init.GPIO_Pins_Top, GPIO_PIN_RESET);


		// 2. Initialize GPIO for UB, VB, WB to GPIO Output mode
		GPIOInit.Pin = pBLDCHandle->Init.GPIO_Pins_Bottom;
		GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
		GPIOInit.Pull = GPIO_NOPULL;
		GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_Init(pBLDCHandle->Init.GPIOx_Bottom, &GPIOInit);
		Delay_ms(10);

		GPIO_WritePin(pBLDCHandle->Init.GPIOx_Bottom, pBLDCHandle->Init.GPIO_Pins_Bottom, GPIO_PIN_RESET);


		// 3. Charge Bootstrap Capacitor for 10ms
		Delay_ms(10);
		GPIO_WritePin(pBLDCHandle->Init.GPIOx_Bottom, pBLDCHandle->Init.GPIO_Pins_Bottom, GPIO_PIN_SET);
		Delay_ms(10);
		GPIO_WritePin(pBLDCHandle->Init.GPIOx_Bottom, pBLDCHandle->Init.GPIO_Pins_Bottom, GPIO_PIN_RESET);



		/********************************************************************
		 *			Low level init EXTI for Hall Sensor interrupt			*
		 ********************************************************************/
		// 1. Configure GPIO of EXTI
		memset(&GPIOInit, 0, sizeof(GPIOInit));

		GPIOInit.Pin = pBLDCHandle->Init.GPIO_Pins_Hall;
		GPIOInit.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIOInit.Pull = GPIO_NOPULL;
		GPIO_Init(pBLDCHandle->Init.GPIOx_Hall, &GPIOInit);

		// 2. Configure NVIC of EXTI
		//NVIC_IRQConfig(IRQ_NO_EXTI9_5, NVIC_PRIOR_8, ENABLE);		// NVIC IRQ for EXTI will be enabled in main function when MotorState becomes 'RUN'



		/********************************************************************
		 *				Low level init TIM to generate PWM signals			*
		 ********************************************************************/
		pBLDCHandle->Init.TIM_Handle = &TIM4Handle;
		pBLDCHandle->Init.TIM_Handle->Instance = TIM4;
		pBLDCHandle->Init.TIM_Handle->Init.CounterMode = TIM_COUNTERMODE_UP;
		pBLDCHandle->Init.TIM_Handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		pBLDCHandle->Init.TIM_Handle->Init.Prescaler = (1-1);	// 72MHz / 1 = 72MHz
		pBLDCHandle->Init.TIM_Handle->Init.Period = (3600-1);	// 72MHz / 3600 = 20kHz
		TIM_PWM_Init(pBLDCHandle->Init.TIM_Handle);

		TIM_OC_InitTypeDef TIM4_PWMConfig;

		memset(&TIM4_PWMConfig, 0, sizeof(TIM4_PWMConfig));

		TIM4_PWMConfig.OCMode = TIM_OCMODE_PWM1;
		TIM4_PWMConfig.OCPolarity = TIM_OCPOLARITY_HIGH;

		TIM4_PWMConfig.Pulse = 0;	// Initially, 0% duty
		TIM_PWM_ConfigChannel(pBLDCHandle->Init.TIM_Handle, &TIM4_PWMConfig, TIM_CHANNEL_1);

		TIM4_PWMConfig.Pulse = 0;	// Initially, 0% duty
		TIM_PWM_ConfigChannel(pBLDCHandle->Init.TIM_Handle, &TIM4_PWMConfig, TIM_CHANNEL_2);

		TIM4_PWMConfig.Pulse = 0;	// Initially, 0% duty
		TIM_PWM_ConfigChannel(pBLDCHandle->Init.TIM_Handle, &TIM4_PWMConfig, TIM_CHANNEL_3);
	}
}


void TFT_MspInit(TFT_HandleTypeDef *pTFTHandle)
{
	GPIO_InitTypeDef GPIOInit;

	memset(&GPIOInit, 0, sizeof(GPIOInit));

	if(pTFTHandle->Instance == TFT1)
	{
		/************************************************************************
		 *		  Low level init GPIO of TFT Control pins and Data pins			*
		 ***********************************************************************/

		// 1. Initialize GPIO for Control Pins (nRST, nCS, RS, nWR) to GPIO Output Push-Pull mode
		pTFTHandle->Init.GPIOx_TFT_Control = GPIOA;
		pTFTHandle->Init.GPIO_Pin_TFT_nRST = GPIO_PIN_8;
		pTFTHandle->Init.GPIO_Pin_TFT_nCS = GPIO_PIN_9;
		pTFTHandle->Init.GPIO_Pin_TFT_RS = GPIO_PIN_10;
		pTFTHandle->Init.GPIO_Pin_TFT_nWR = GPIO_PIN_11;
		pTFTHandle->Init.GPIO_Pins_TFT_Control = pTFTHandle->Init.GPIO_Pin_TFT_nRST | pTFTHandle->Init.GPIO_Pin_TFT_nCS | pTFTHandle->Init.GPIO_Pin_TFT_RS | pTFTHandle->Init.GPIO_Pin_TFT_nWR;

		GPIOInit.Pin = pTFTHandle->Init.GPIO_Pins_TFT_Control;
		GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
		GPIOInit.Pull = GPIO_NOPULL;
		GPIOInit.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_Init(pTFTHandle->Init.GPIOx_TFT_Control, &GPIOInit);
		Delay_ms(10);

		GPIO_ModifyPin(pTFTHandle->Init.GPIOx_TFT_Control, pTFTHandle->Init.GPIO_Pin_TFT_nRST | pTFTHandle->Init.GPIO_Pin_TFT_nCS | pTFTHandle->Init.GPIO_Pin_TFT_nWR, pTFTHandle->Init.GPIO_Pin_TFT_RS);

		// 2. Initialize GPIO for Data Pins (D0 ~ D15) to GPIO Output mode
		pTFTHandle->Init.GPIOx_TFT_Data = GPIOC;
		pTFTHandle->Init.GPIO_Pins_TFT_Data = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

		GPIOInit.Pin = pTFTHandle->Init.GPIO_Pins_TFT_Data;
		GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
		GPIOInit.Pull = GPIO_NOPULL;
		GPIOInit.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_Init(pTFTHandle->Init.GPIOx_TFT_Data, &GPIOInit);
		Delay_ms(10);

		GPIO_WritePort(pTFTHandle->Init.GPIOx_TFT_Data, pTFTHandle->Init.GPIO_Pins_TFT_Data, GPIO_PIN_RESET);
	}
}


void TS_MspInit(TS_HandleTypeDef *pTSHandle)
{
	GPIO_InitTypeDef GPIOInit;

	memset(&GPIOInit, 0, sizeof(GPIOInit));

	if(pTSHandle->Instance == TS1)
	{
		/************************************************************************
		 *					Low level init GPIO of TS_nCS, TS_nINT				*
		 ***********************************************************************/
		pTSHandle->Init.GPIOx_TS_Control = GPIOB;
		pTSHandle->Init.GPIO_Pin_TS_nCS = GPIO_PIN_3;
		pTSHandle->Init.GPIO_Pin_TS_nINT = GPIO_PIN_4;

		// Initialize GPIO for TS_nCS to GPIO Output mode
		GPIOInit.Pin = pTSHandle->Init.GPIO_Pin_TS_nCS;
		GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
		GPIOInit.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_Init(pTSHandle->Init.GPIOx_TS_Control, &GPIOInit);
		Delay_ms(10);

		GPIO_WritePin(pTSHandle->Init.GPIOx_TS_Control, pTSHandle->Init.GPIO_Pin_TS_nCS, GPIO_PIN_SET); // Initially, TS_nCS = SET

		memset(&GPIOInit, 0, sizeof(GPIOInit));

		// Initialize GPIO for TS_nINT to GPIO Input floating mode
		GPIOInit.Pin = pTSHandle->Init.GPIO_Pin_TS_nINT;
		GPIOInit.Mode = GPIO_MODE_INPUT;
		GPIOInit.Pull = GPIO_NOPULL;
		GPIO_Init(pTSHandle->Init.GPIOx_TS_Control, &GPIOInit);
		Delay_ms(10);

		GPIO_WritePin(pTSHandle->Init.GPIOx_TS_Control, pTSHandle->Init.GPIO_Pin_TS_nINT, GPIO_PIN_RESET);


		/********************************************************************
		 *			Low level init SPI to get Touch Screen coordinates		*
		 ********************************************************************/
		pTSHandle->Init.SPI_Handle = &SPI2Handle;
		pTSHandle->Init.SPI_Handle->Instance = SPI2;
		pTSHandle->Init.SPI_Handle->Init.Mode = SPI_MODE_MASTER;
		pTSHandle->Init.SPI_Handle->Init.Direction = SPI_DIRECTION_2LINES;
		pTSHandle->Init.SPI_Handle->Init.DataSize = SPI_DATASIZE_16BIT;
		pTSHandle->Init.SPI_Handle->Init.CLKPolarity = SPI_POLARITY_LOW;
		pTSHandle->Init.SPI_Handle->Init.CLKPhase = SPI_PHASE_1EDGE;
		pTSHandle->Init.SPI_Handle->Init.NSS = SPI_NSS_SOFT;
		pTSHandle->Init.SPI_Handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
		pTSHandle->Init.SPI_Handle->Init.FirstBit = SPI_FIRSTBIT_MSB;
		pTSHandle->Init.SPI_Handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

		SPI_Init(pTSHandle->Init.SPI_Handle);
	}
}


void SPI_MspInit(SPI_HandleTypeDef *pSPIHandle)
{
	// 1. Configure GPIO for SPI
	GPIO_HandleTypeDef GPIOHandle;

	memset(&GPIOHandle, 0, sizeof(GPIOHandle));

	if(pSPIHandle->Instance == SPI2)
	{
		// SPI2_SCK, SPI2_MOSI
		GPIOHandle.Instance = GPIOB;
		GPIOHandle.Init.Pin = GPIO_PIN_13 | GPIO_PIN_15;
		GPIOHandle.Init.Mode = GPIO_MODE_AF_PP;
		GPIOHandle.Init.Speed = GPIO_SPEED_FREQ_HIGH;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);

		memset(&GPIOHandle, 0, sizeof(GPIOHandle));

		// SPI2_MISO
		GPIOHandle.Instance = GPIOB;
		GPIOHandle.Init.Pin = GPIO_PIN_14;
		GPIOHandle.Init.Mode = GPIO_MODE_INPUT;
		GPIOHandle.Init.Pull = GPIO_NOPULL;

		GPIO_Init(GPIOHandle.Instance, &GPIOHandle.Init);
	}

	// 2. Configure CLOCK for SPI
	SPI_PeripheralClockControl(pSPIHandle->Instance, ENABLE);
}
