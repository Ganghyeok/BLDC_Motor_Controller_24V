/*
 * stm32f103xx_rcc_driver.c
 *
 *  Created on: Dec 2, 2020
 *      Author: Ganghyeok Lim
 */

#include "stm32f103xx_rcc_driver.h"


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct)
{
	/*------------------------------- HSE Configuration ------------------------*/
	if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
	{
		if(RCC_OscInitStruct->HSEState == RCC_HSE_ON)
		{

			SET_BIT(RCC->CR, RCC_CR_HSEON);				// Enable HSE
			WAIT_FLAG_SET(RCC->CR, RCC_CR_HSERDY);		// Wait until HSERDY flag is set
		}

	}

	/*-------------------------------- PLL Configuration -----------------------*/
	if ((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON)
	{
		// 1. Disable PLL
		CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

		// 2. Check whether PLLSOURCE is HSE or not
		if (RCC_OscInitStruct->PLL.PLLSource == RCC_PLLSOURCE_HSE)
		{
			// Configure HSEPredivValue
			CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLXTPRE);
		}

		// 3. Configure PLL source and PLL multiplication factor
		MODIFY_REG( RCC->CFGR, ( (RCC_CFGR_PLLSRC) | (RCC_CFGR_PLLMULL) ), ( (RCC_PLLSOURCE_HSE) | (RCC_OscInitStruct->PLL.PLLMUL) ) );

		// 4. Enable PLL
		SET_BIT(RCC->CR, RCC_CR_PLLON);

		// 5. Wait until PLL is ready
		WAIT_FLAG_SET(RCC->CR, RCC_CR_PLLRDY);
	}

}


void RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency)
{
	// 1. Configure FLASH Latency
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLatency);

	// 2. Configure APB prescaler
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_ClkInitStruct->APB2CLKDivider);

	// 3. Configure AHB prescaler
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);

	// 4. Configure SYSCLK
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_ClkInitStruct->SYSCLKSource);

	// 5. Wait until SYSCLK is PLLCLK
	WAIT_FLAG_SET(RCC->CFGR, RCC_CFGR_SWS_PLL);
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


uint32_t RCC_GetPLLOutputClock(void)
{
	uint32_t PLLInputClock;
	uint8_t PLLMulFactor;
	uint8_t tmp;
	uint32_t PLLOutputClock;

	// 1. Check PLL On
	if( !((RCC->CR & RCC_CR_PLLON) && (RCC->CR & RCC_CR_PLLRDY)) )
	{
		// When PLL is Off
		return 0;
	}

	// 2. Check PLL source
	if( RCC->CFGR & RCC_CFGR_PLLSRC )
	{
		// PLL source is HSE oscillator clock
		PLLInputClock = 8000000U;
	}
	else
	{
		// PLL source is HSI oscillator clock / 2
		PLLInputClock = (8000000U / 2);
	}

	// 3. Check PLL multiplication factor
	tmp = (RCC->CFGR & RCC_CFGR_PLLMULL) >> 18;

	if(tmp == 0xF)
	{
		PLLMulFactor = 16;
	}
	else
	{
		PLLMulFactor = tmp + 2;
	}

	// 4. Calculate PLL output clock
	PLLOutputClock = PLLInputClock * PLLMulFactor;


	return PLLOutputClock;
}



uint32_t RCC_GetPCLKxValue(uint8_t pclkType)
{
	uint8_t sysclkSrcType;
	uint32_t sysclkValue;
	uint32_t pclkValue;

	// 1. Check current system clock
	sysclkSrcType = (RCC->CFGR >> 2) & 0x3;

	if(sysclkSrcType == 0)
	{
		// System clock source is HSI
		sysclkValue = 8000000;
	}
	else if(sysclkSrcType == 1)
	{
		// System clock source is HSE
		sysclkValue = 8000000;
	}
	else if(sysclkSrcType == 2)
	{
		// System clock source is PLL output
		sysclkValue = RCC_GetPLLOutputClock();
	}

	// 2. Check AHB prescaler value
	uint8_t ahbPrsc;
	uint16_t ahbPrscTable[8] = {2, 4, 8, 16, 64, 128, 256, 512};
	uint8_t tmp;

	tmp = (RCC->CFGR >> 4) & 0xF;

	if(tmp < 8)
	{
		ahbPrsc = 1;
	}
	else
	{
		ahbPrsc = ahbPrscTable[(tmp-8)];
	}

	// 3. Check APB prescaler value
	uint8_t apb1Prsc, apb2Prsc;
	uint8_t apbPrscTable[4] = {2, 4, 8, 16};

	if(pclkType == PCLK1)
	{
		tmp = (RCC->CFGR >> 8) & 0x7;

		if(tmp < 4)		apb1Prsc = 1;
		else			apb1Prsc = apbPrscTable[(tmp - 4)];
	}
	else if(pclkType == PCLK2)
	{
		tmp = (RCC->CFGR >> 11) & 0x7;

		if(tmp < 4)		apb2Prsc = 1;
		else			apb2Prsc = apbPrscTable[(tmp - 4)];

	}

	// 4. Calculate System clock value
	if(pclkType == PCLK1)			pclkValue = (sysclkValue / ahbPrsc) / apb1Prsc;
	else if(pclkType == PCLK2)		pclkValue = (sysclkValue / ahbPrsc) / apb2Prsc;


	return pclkValue; // [MHz]
}


























