/*
 * common.c
 *
 *  Created on: 2021. 1. 3.
 *      Author: Ganghyeok Lim
 */

#include "common.h"


/********************************************************************************************************************
 * 																											  		*
 *												User Common Function												*
 * 																											  		*
 ********************************************************************************************************************/

void NVIC_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(IRQNumber < 32)
		{
			// IRQ0 ~ IRQ31
			NVIC->ISER[0] = (uint32_t)(1UL << (uint32_t)IRQNumber);
		}
		else if(IRQNumber < 60)
		{
			// IRQ32 ~ IRQ63
			NVIC->ISER[1] = (uint32_t)(1UL << (uint32_t)(IRQNumber % 32));
		}

	}
	else if(En_or_Di == DISABLE)
	{
		if(IRQNumber < 32)
		{
			// IRQ0 ~ IRQ31
			NVIC->ICER[0] = (uint32_t)(1UL << (uint32_t)IRQNumber);
		}
		else if(IRQNumber < 60)
		{
			// IRQ32 ~ IRQ63
			NVIC->ICER[1] = (uint32_t)(1UL << (uint32_t)(IRQNumber % 32));
		}
	}

	// IRQ Priority configuration
	NVIC->IPR[IRQNumber] = (IRQPriority << 4UL);
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

