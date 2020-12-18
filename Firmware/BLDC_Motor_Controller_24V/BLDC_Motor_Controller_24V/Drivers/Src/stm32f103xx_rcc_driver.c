/*
 * stm32f103xx_rcc_driver.c
 *
 *  Created on: Dec 18, 2020
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



void RCC_MCOConfig(uint32_t MCO_Option)
{
	GPIO_InitTypeDef GPIO_MCOConfig;

	memset(&GPIO_MCOConfig, 0, sizeof(GPIO_MCOConfig));

	GPIO_MCOConfig.Mode = GPIO_MODE_AF_PP;
	GPIO_MCOConfig.Pin = GPIO_PIN_8;
	GPIO_MCOConfig.Pull = GPIO_NOPULL;
	GPIO_MCOConfig.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_Init(GPIOA, &GPIO_MCOConfig);

	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, MCO_Option);
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
