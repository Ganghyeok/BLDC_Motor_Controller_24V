/*
 * stm32f103xx_tim_driver.c
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#include "stm32f103xx_tim_driver.h"


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void TIM_Base_Init(TIM_HandleTypeDef *pTIMHandle)
{
	// 1. Check state of TIMx is RESET
	if(pTIMHandle->State != TIM_STATE_RESET)
	{
		// State of TIMx is not RESET
		return;
	}

	// 2. Init Low level hardware of TIM : GPIO, CLOCK
	TIM_Base_MspInit(pTIMHandle->Instance);

	// 3. Set the Time Base configuration
	TIM_Base_SetConfig(pTIMHandle);

	// 4. Init the TIM state
	pTIMHandle->State = TIM_STATE_READY;
}



__weak void TIM_Base_MspInit(TIM_TypeDef *TIMx)
{
	/* Prevent unused argument(s) compilation warning */
		UNUSED(TIMx);

	/* NOTE : This function should not be modified, when the callback is needed,
	 * 		  the TIM_MspInit could be implemented in the user file
	 * 		  (This is a weak implementation. The user application may override this function)
	 */
}



void TIM_PWM_Init(TIM_HandleTypeDef *pTIMHandle)
{
	// 1. Check state of TIMx is RESET
	if(pTIMHandle->State != TIM_STATE_RESET)
	{
		// State of TIMx is not RESET
		return;
	}

	// 2. Init Low level hardware of TIM : GPIO, CLOCK
	TIM_PWM_MspInit(pTIMHandle);

	// 3. Set the Time Base configuration
	TIM_Base_SetConfig(pTIMHandle);

	// 4. Init the TIM state
	pTIMHandle->State = TIM_STATE_READY;
}


__weak void TIM_PWM_MspInit(TIM_HandleTypeDef *pTIMHandle)
{
	/* Prevent unused argument(s) compilation warning */
		UNUSED(pTIMHandle);

	/* NOTE : This function should not be modified, when the callback is needed,
	 * 		  the TIM_PWM_MspInit could be implemented in the user file
	 * 		  (This is a weak implementation. The user application may override this function)
	 */
}


void TIM_Base_SetConfig(TIM_HandleTypeDef *pTIMHandle)
{
	uint32_t temp = 0;

	// 1. Configure counter mode
	temp |= pTIMHandle->Init.CounterMode;

	// 2. Decide the use of Auto-reload preload
	temp |= pTIMHandle->Init.AutoReloadPreload;

	MODIFY_REG(pTIMHandle->Instance->CR1, (TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_ARPE), temp);

	// 3. Configure ARR value
	pTIMHandle->Instance->ARR = (uint32_t)pTIMHandle->Init.Period;

	// 4. Configure Prescaler value
	pTIMHandle->Instance->PSC = pTIMHandle->Init.Prescaler;

	// 5. Configure Repetition counter value
	pTIMHandle->Instance->RCR = pTIMHandle->Init.RepetitionCounter;

	// 6. Generate update event to reload some registers
	pTIMHandle->Instance->EGR |= TIM_EGR_UG;
}


void TIM_PWM_ConfigChannel(TIM_HandleTypeDef *pTIMHandle, TIM_OC_InitTypeDef *sConfig, uint32_t Channel)
{
	switch (Channel)
	{
		case TIM_CHANNEL_1:
		{
			// 1. Disable the channel
			pTIMHandle->Instance->CCER &= ~TIM_CCER_CC1E;

			// 2. Reset the output compare mode bits and Select the Output Compare Mode
			MODIFY_REG(pTIMHandle->Instance->CCMR1, (TIM_CCMR1_CC1S | TIM_CCMR1_OC1M), sConfig->OCMode);

			// 3. Reset the Output Polarity level and Set the Output Compare Polarity
			MODIFY_REG(pTIMHandle->Instance->CCER, TIM_CCER_CC1P, sConfig->OCPolarity);

			// 4. Set the Capture Compare Register value
			pTIMHandle->Instance->CCR1 = sConfig->Pulse;

			// 5. Set the Preload enable bit for channel1
			pTIMHandle->Instance->CCMR1 |= TIM_CCMR1_OC1PE;

			break;
		}

		case TIM_CHANNEL_2:
		{
			// 1. Disable the channel
			pTIMHandle->Instance->CCER &= ~TIM_CCER_CC2E;

			// 2. Reset the output compare mode bits and Select the Output Compare Mode
			MODIFY_REG(pTIMHandle->Instance->CCMR1, (TIM_CCMR1_CC2S | TIM_CCMR1_OC2M), (sConfig->OCMode << 8U));

			// 3. Reset the Output Polarity level and Set the Output Compare Polarity
			MODIFY_REG(pTIMHandle->Instance->CCER, TIM_CCER_CC2P, (sConfig->OCPolarity << 4U));

			// 4. Set the Capture Compare Register value
			pTIMHandle->Instance->CCR2 = sConfig->Pulse;

			// 5. Set the Preload enable bit for channel1
			pTIMHandle->Instance->CCMR1 |= TIM_CCMR1_OC2PE;

			break;
		}

		case TIM_CHANNEL_3:
		{
			// 1. Disable the channel
			pTIMHandle->Instance->CCER &= ~TIM_CCER_CC3E;

			// 2. Reset the output compare mode bits and Select the Output Compare Mode
			MODIFY_REG(pTIMHandle->Instance->CCMR2, (TIM_CCMR2_CC3S | TIM_CCMR2_OC3M), sConfig->OCMode);

			// 3. Reset the Output Polarity level and Set the Output Compare Polarity
			MODIFY_REG(pTIMHandle->Instance->CCER, TIM_CCER_CC3P, (sConfig->OCPolarity << 8U));

			// 4. Set the Capture Compare Register value
			pTIMHandle->Instance->CCR3 = sConfig->Pulse;

			// 5. Set the Preload enable bit for channel1
			pTIMHandle->Instance->CCMR2 |= TIM_CCMR2_OC3PE;

			break;
		}

		case TIM_CHANNEL_4:
		{
			// 1. Disable the channel
			pTIMHandle->Instance->CCER &= ~TIM_CCER_CC4E;

			// 2. Reset the output compare mode bits and Select the Output Compare Mode
			MODIFY_REG(pTIMHandle->Instance->CCMR2, (TIM_CCMR2_CC4S | TIM_CCMR2_OC4M), (sConfig->OCMode << 8U));

			// 3. Reset the Output Polarity level and Set the Output Compare Polarity
			MODIFY_REG(pTIMHandle->Instance->CCER, TIM_CCER_CC4P, (sConfig->OCPolarity << 12U));

			// 4. Set the Capture Compare Register value
			pTIMHandle->Instance->CCR4 = sConfig->Pulse;

			// 5. Set the Preload enable bit for channel1
			pTIMHandle->Instance->CCMR2 |= TIM_CCMR2_OC4PE;

			break;
		}
		default :
			break;
	}
}


void TIM_PeripheralClockControl(TIM_TypeDef *TIMx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(TIMx == TIM1)		RCC_TIM1_CLK_ENABLE();
		else if(TIMx == TIM2)	RCC_TIM2_CLK_ENABLE();
		else if(TIMx == TIM3)	RCC_TIM3_CLK_ENABLE();
		else if(TIMx == TIM4)	RCC_TIM4_CLK_ENABLE();
		else if(TIMx == TIM5)	RCC_TIM5_CLK_ENABLE();
		else if(TIMx == TIM6)	RCC_TIM6_CLK_ENABLE();
		else if(TIMx == TIM7)	RCC_TIM7_CLK_ENABLE();
		else if(TIMx == TIM8)	RCC_TIM8_CLK_ENABLE();
	}
	else if(En_or_Di == DISABLE)
	{
		if(TIMx == TIM1)		RCC_TIM1_CLK_DISABLE();
		else if(TIMx == TIM2)	RCC_TIM2_CLK_DISABLE();
		else if(TIMx == TIM3)	RCC_TIM3_CLK_DISABLE();
		else if(TIMx == TIM4)	RCC_TIM4_CLK_DISABLE();
		else if(TIMx == TIM5)	RCC_TIM5_CLK_DISABLE();
		else if(TIMx == TIM6)	RCC_TIM6_CLK_DISABLE();
		else if(TIMx == TIM7)	RCC_TIM7_CLK_DISABLE();
		else if(TIMx == TIM8)	RCC_TIM8_CLK_DISABLE();
	}
}


void TIM_PWM_Start(TIM_HandleTypeDef *pTIMHandle, uint32_t Channel)
{
	// Enable the channel
	if(Channel == TIM_CHANNEL_1)		pTIMHandle->Instance->CCER |= TIM_CCER_CC1E;
	else if(Channel == TIM_CHANNEL_2)	pTIMHandle->Instance->CCER |= TIM_CCER_CC2E;
	else if(Channel == TIM_CHANNEL_3)	pTIMHandle->Instance->CCER |= TIM_CCER_CC3E;
	else if(Channel == TIM_CHANNEL_4)	pTIMHandle->Instance->CCER |= TIM_CCER_CC4E;

	// Enable the Main output
	pTIMHandle->Instance->BDTR |= TIM_BDTR_MOE;

	// Enable the TIM1
	pTIMHandle->Instance->CR1 |= TIM_CR1_CEN;
}



void TIM_IRQHandling(TIM_HandleTypeDef *pTIMHandle)
{
	/* Interrupt handling for TIM */

	uint32_t temp1, temp2;

	// 1. Handle for interrupt generated by Update Event
	temp1 = READ_BIT(pTIMHandle->Instance->SR, TIM_SR_UIF);
	temp2 = READ_BIT(pTIMHandle->Instance->DIER, TIM_DIER_UIE);

	if(temp1 && temp2)
	{
		// This interrupt is generated by Update Event
		CLEAR_FLAG(TIM6->SR, TIM_SR_UIF);

		TIM_PeriodElapsedCallback(pTIMHandle);
	}
}



__weak void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pTIMHandle)
{
	/* Prevent unused argument(s) compilation warning */
		UNUSED(pTIMHandle);

	/* NOTE : This function should not be modified, when the callback is needed,
	 * 		  the TIM_PeriodElapsedCallback could be implemented in the user file
	 * 		  (This is a weak implementation. The user application may override this function)
	 */
}
