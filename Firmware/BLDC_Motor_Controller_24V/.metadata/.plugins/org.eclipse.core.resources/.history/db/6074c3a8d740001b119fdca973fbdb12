/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: 2020. 12. 4.
 *      Author: Ganghyeok Lim
 */

#include "stm32f103xx_gpio_driver.h"


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

/* Initialization and de-initialization functions *****************************/
void GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
	uint32_t config = 0x0U;
	volatile uint32_t *configRegister;
	uint32_t registerOffset = 0;

	// GPIO Clock enable
	GPIOx_ClockEnable(GPIOx);

	for(uint8_t position = 0; position < 16; position++)
	{
		if( GPIO_Init->Pin & (1 << position) )
		{
			/*------------------------- GPIO Port Configuration --------------------*/

			switch(GPIO_Init->Mode)
			{
				case GPIO_MODE_OUTPUT_PP :
				{
					// Configure MODE field to output mode with max speed
					if(GPIO_Init->Speed == GPIO_SPEED_FREQ_LOW)				config |= (0x1 << 1);
					else if(GPIO_Init->Speed == GPIO_SPEED_FREQ_MEDIUM)		config |= (0x1 << 0);
					else if(GPIO_Init->Speed == GPIO_SPEED_FREQ_HIGH)		config |= (0x3 << 0);

					// Configure CNF field to general purpose output push-pull
					config &= ~(0x3 << 2);


					break;
				}

				case GPIO_MODE_OUTPUT_OD :
				{
					// Configure MODE field to output mode with max speed
					if(GPIO_Init->Speed == GPIO_SPEED_FREQ_LOW)				config |= (0x1 << 1);
					else if(GPIO_Init->Speed == GPIO_SPEED_FREQ_MEDIUM)		config |= (0x1 << 0);
					else if(GPIO_Init->Speed == GPIO_SPEED_FREQ_HIGH)		config |= (0x3 << 0);

					// Configure CNF field to general purpose output open-drain
					config |= (0x1 << 2);

					break;
				}

				case GPIO_MODE_AF_PP :
				{
					// Configure MODE field to output mode with max speed
					if(GPIO_Init->Speed == GPIO_SPEED_FREQ_LOW)				config |= (0x1 << 1);
					else if(GPIO_Init->Speed == GPIO_SPEED_FREQ_MEDIUM)		config |= (0x1 << 0);
					else if(GPIO_Init->Speed == GPIO_SPEED_FREQ_HIGH)		config |= (0x3 << 0);

					// Configure CNF field to alternate function output push-pull
					config |= (0x1 << 3);

					break;
				}

				case GPIO_MODE_AF_OD :
				{
					// Configure MODE field to output mode with max speed
					if(GPIO_Init->Speed == GPIO_SPEED_FREQ_LOW)				config |= (0x1 << 1);
					else if(GPIO_Init->Speed == GPIO_SPEED_FREQ_MEDIUM)		config |= (0x1 << 0);
					else if(GPIO_Init->Speed == GPIO_SPEED_FREQ_HIGH)		config |= (0x3 << 0);

					// Configure CNF field to alternate function output open-drain
					config |= (0x3 << 2);

					break;
				}

				case GPIO_MODE_INPUT :
				case GPIO_MODE_IT_RISING :
				case GPIO_MODE_IT_FALLING :
				case GPIO_MODE_IT_RISING_FALLING :
				{
					// Configure MODE field to input mode
					config &= ~(0x3 << 0);

					if(GPIO_Init->Pull == GPIO_NOPULL)
					{
						config |= (0x1 << 2);
					}
					else if(GPIO_Init->Pull == GPIO_PULLUP)
					{
						config |= (0x1 << 3);
						GPIOx->BSRR |= (0x1 << position);
					}
					else if(GPIO_Init->Pull == GPIO_PULLDOWN)
					{
						config |= (0x1 << 3);
						GPIOx->BRR |= (0x1 << position);
					}

					break;
				}

			}


			configRegister = (position < 8) ? &(GPIOx->CRL) : &(GPIOx->CRH);
			registerOffset = (position < 8) ? (4 * position) : (4 * (position - 8));

			MODIFY_REG( (*configRegister), (0xF << registerOffset), (config << registerOffset) );


			/*------------------------- EXTI Mode Configuration --------------------*/
			if( (GPIO_Init->Mode & EXTI_MODE) == EXTI_MODE )
			{
				RCC_AFIO_CLK_ENABLE();

				int exti_sel, exti_pos;
				exti_sel = position / 4;
				exti_pos = position % 4;

				MODIFY_REG(AFIO->EXTICR[exti_sel], (0xF << exti_pos), (GET_GPIOCODE(GPIOx) << exti_pos));

				/* Configure the interrupt mask */
				if( (GPIO_Init->Mode & GPIO_MODE_IT) == GPIO_MODE_IT )
				{
					SET_BIT(EXTI->IMR, (1 << position));
				}
				else
				{
					CLEAR_BIT(EXTI->IMR, (1 << position));
				}

				/* Configure the event mask */
				if( (GPIO_Init->Mode & GPIO_MODE_EVT) == GPIO_MODE_EVT )
				{
					SET_BIT(EXTI->EMR, (1 << position));
				}
				else
				{
					CLEAR_BIT(EXTI->EMR, (1 << position));
				}

				/* Enable or disable the rising trigger */
				if( (GPIO_Init->Mode & RISING_EDGE) == RISING_EDGE )
				{
					SET_BIT(EXTI->RTSR, (1 << position));
				}
				else
				{
					CLEAR_BIT(EXTI->RTSR, (1 << position));
				}

				/* Enable or disable the falling trigger */
				if( (GPIO_Init->Mode & FALLING_EDGE) == FALLING_EDGE )
				{
					SET_BIT(EXTI->FTSR, (1 << position));
				}
				else
				{
					CLEAR_BIT(EXTI->FTSR, (1 << position));
				}

			}

		}

		config = 0x0;
		registerOffset = 0;
	}
}



void GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{

}



/* IO operation functions *****************************************************/
uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	if((uint16_t)GPIOx->IDR & GPIO_Pin)		return GPIO_PIN_SET;
	else									return GPIO_PIN_RESET;
}



void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t PinState)
{
	if(PinState == GPIO_PIN_SET)
	{
		GPIOx->BSRR |= GPIO_Pin;
	}
	else
	{
		GPIOx->BSRR |= ((uint32_t)GPIO_Pin << 16);
	}
}



void GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	if( (GPIOx->ODR & GPIO_Pin) )
	{
		// State of pin was LOW
		GPIOx->BSRR |= ((uint32_t)GPIO_Pin << 16);
	}
	else
	{
		// State of pin was HIGH
		GPIOx->BSRR |= GPIO_Pin;
	}
}








