/*
 * user_func.c
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#include "main.h"


void NVIC_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(IRQNumber < 32)
		{
			// IRQ0 ~ IRQ31
			NVIC->ISER[0] |= (1 << IRQNumber);
		}
		else if(IRQNumber < 60)
		{
			// IRQ32 ~ IRQ63
			NVIC->ISER[1] |= (1 << (IRQNumber % 32));
		}

	}
	else if(En_or_Di == DISABLE)
	{
		if(IRQNumber < 32)
		{
			// IRQ0 ~ IRQ31
			NVIC->ICER[0] |= (1 << IRQNumber);
		}
		else if(IRQNumber < 60)
		{
			// IRQ32 ~ IRQ63
			NVIC->ICER[1] |= (1 << (IRQNumber % 32));
		}
	}

	// IRQ Priority configuration
	NVIC->IPR[IRQNumber] |= (IRQPriority << 4);
}


void SystemClock_Config(uint8_t clockFreq)
{
	RCC_OscInitTypeDef oscInit;
	RCC_ClkInitTypeDef clkInit;

	uint8_t FLatency = 0;

	memset(&oscInit, 0, sizeof(oscInit));
	memset(&clkInit, 0, sizeof(clkInit));

	oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	oscInit.HSEState = RCC_HSE_ON;
	oscInit.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	oscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	oscInit.PLL.PLLState = RCC_PLL_ON;

	switch(clockFreq)
	{
		case SYSCLK_FREQ_16MHZ :
		{
			oscInit.PLL.PLLMUL = RCC_PLL_MUL2;

			clkInit.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
			clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;	// 16MHz
			clkInit.APB1CLKDivider = RCC_HCLK_DIV1;		// 16MHz
			clkInit.APB2CLKDivider = RCC_HCLK_DIV1;		// 16MHz

			FLatency = FLASH_LATENCY_0;

			break;
		}

		case SYSCLK_FREQ_24MHZ :
		{
			oscInit.PLL.PLLMUL = RCC_PLL_MUL3;

			clkInit.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
			clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;	// 24MHz
			clkInit.APB1CLKDivider = RCC_HCLK_DIV1;		// 24MHz
			clkInit.APB2CLKDivider = RCC_HCLK_DIV1;		// 24MHz

			FLatency = FLASH_LATENCY_0;

			break;
		}

		case SYSCLK_FREQ_32MHZ :
		{
			oscInit.PLL.PLLMUL = RCC_PLL_MUL4;

			clkInit.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
			clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;	// 32MHz
			clkInit.APB1CLKDivider = RCC_HCLK_DIV1;		// 32MHz
			clkInit.APB2CLKDivider = RCC_HCLK_DIV1;		// 32MHz

			FLatency = FLASH_LATENCY_1;

			break;
		}

		case SYSCLK_FREQ_40MHZ :
		{
			oscInit.PLL.PLLMUL = RCC_PLL_MUL5;

			clkInit.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
			clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;	// 40MHz
			clkInit.APB1CLKDivider = RCC_HCLK_DIV2;		// 20MHz
			clkInit.APB2CLKDivider = RCC_HCLK_DIV1;		// 40MHz

			FLatency = FLASH_LATENCY_1;

			break;
		}

		case SYSCLK_FREQ_48MHZ :
		{
			oscInit.PLL.PLLMUL = RCC_PLL_MUL6;

			clkInit.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
			clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;	// 48MHz
			clkInit.APB1CLKDivider = RCC_HCLK_DIV2;		// 24MHz
			clkInit.APB2CLKDivider = RCC_HCLK_DIV1;		// 48MHz

			FLatency = FLASH_LATENCY_1;

			break;
		}

		case SYSCLK_FREQ_56MHZ :
		{
			oscInit.PLL.PLLMUL = RCC_PLL_MUL7;

			clkInit.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
			clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;	// 56MHz
			clkInit.APB1CLKDivider = RCC_HCLK_DIV2;		// 28MHz
			clkInit.APB2CLKDivider = RCC_HCLK_DIV1;		// 56MHz

			FLatency = FLASH_LATENCY_2;

			break;
		}

		case SYSCLK_FREQ_64MHZ :
		{
			oscInit.PLL.PLLMUL = RCC_PLL_MUL8;

			clkInit.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
			clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;	// 64MHz
			clkInit.APB1CLKDivider = RCC_HCLK_DIV2;		// 32MHz
			clkInit.APB2CLKDivider = RCC_HCLK_DIV1;		// 64MHz

			FLatency = FLASH_LATENCY_2;

			break;
		}

		case SYSCLK_FREQ_72MHZ :
		{
			oscInit.PLL.PLLMUL = RCC_PLL_MUL9;

			clkInit.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
			clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;	// 72MHz
			clkInit.APB1CLKDivider = RCC_HCLK_DIV2;		// 36MHz
			clkInit.APB2CLKDivider = RCC_HCLK_DIV1;		// 72MHz

			FLatency = FLASH_LATENCY_2;

			break;
		}

		default :
		{
			break;
		}

	}

	RCC_OscConfig(&oscInit);

	RCC_ClockConfig(&clkInit, (uint32_t)FLatency);
}


void Delay_us(uint32_t time_us)
{
	register uint32_t i, j;

	for(i = 0; i < (time_us / 10); i++)
	{
		for(j = 0; j < 0x4D; j++)
		{
			asm volatile ("NOP");
		}
	}
}


void Delay_ms(uint32_t time_ms)
{
	Delay_us(time_ms * 1000);
}


void GPIO_BLDC_Init(void)
{
	GPIO_InitTypeDef GPIOInit;

	memset(&GPIOInit, 0, sizeof(GPIOInit));

	// 1. Initialize GPIO for UT, VT, WT to GPIO Output mode
	GPIOInit.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOInit.Pull = GPIO_NOPULL;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(GPIOB, &GPIOInit);

	GPIO_WritePin(GPIOB, (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15), GPIO_PIN_RESET);

	// 2. Initialize GPIO for UB, VB, WB to GPIO Output mode
	GPIOInit.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
	GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOInit.Pull = GPIO_NOPULL;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(GPIOA, &GPIOInit);

	GPIO_WritePin(GPIOA, (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10), GPIO_PIN_RESET);

	// 3. Charge Bootstrap Capacitor for 10ms
	Delay_ms(10);
	GPIO_WritePin(GPIOA, (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10), GPIO_PIN_SET);
	Delay_ms(10);
	GPIO_WritePin(GPIOA, (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10), GPIO_PIN_RESET);
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

	TIM3_PWMConfig.Pulse = 2;	// (2/10)*100 = 20% duty
	TIM_PWM_ConfigChannel(pTIMHandle, &TIM3_PWMConfig, TIM_CHANNEL_1);

	TIM3_PWMConfig.Pulse = 4;	// (4/10)*100 = 40% duty
	TIM_PWM_ConfigChannel(pTIMHandle, &TIM3_PWMConfig, TIM_CHANNEL_2);

	TIM3_PWMConfig.Pulse = 6;	// (6/10)*100 = 60% duty
	TIM_PWM_ConfigChannel(pTIMHandle, &TIM3_PWMConfig, TIM_CHANNEL_3);

	TIM3_PWMConfig.Pulse = 8;	// (8/10)*100 = 80% duty
	TIM_PWM_ConfigChannel(pTIMHandle, &TIM3_PWMConfig, TIM_CHANNEL_4);
}


void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pTIMHandle)
{
	if(pTIMHandle->Instance == TIM6)
	{
		GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	}
}


void EXTI_Init(GPIO_HandleTypeDef *GPIOHandle)
{
	GPIOHandle->Instance = GPIOC;
	GPIOHandle->Init.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
	GPIOHandle->Init.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIOHandle->Init.Pull = GPIO_NOPULL;
	GPIO_Init(GPIOHandle->Instance, &GPIOHandle->Init);

	NVIC_IRQConfig(IRQ_NO_EXTI9_5, NVIC_PRIOR_8, ENABLE);
}


void EXTI_Callback(uint32_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);

	uint32_t HallPhase;

	HallPhase = (READ_BIT(GPIOC->IDR, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8)) >> 6U;

	switch (HallPhase)
	{
		case Phase1:
		{
			BLDC_Step2();
			break;
		}

		case Phase2:
		{
			BLDC_Step1();
			break;
		}

		case Phase3:
		{
			BLDC_Step6();
			break;
		}

		case Phase4:
		{
			BLDC_Step5();
			break;
		}

		case Phase5:
		{
			BLDC_Step4();
			break;
		}

		case Phase6:
		{
			BLDC_Step3();
			break;
		}

		default :
			break;
	}
}
