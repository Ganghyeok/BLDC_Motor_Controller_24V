/*
 * ts.c
 *
 *  Created on: Jan 16, 2021
 *      Author: Ganghyeok Lim
 */

#include "stm32f103xx.h"
#include "main.h"


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void TS_Init(TS_HandleTypeDef *pTSHandle)
{
	// Init the Low level hardware of Touch Screen : GPIO, SPI
	TS_MspInit(pTSHandle);
}


uint16_t TS_Read_ADS7846(uint16_t command)
{
	uint16_t axis;

	axis = SPI2->DR;
	GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	SPI2->DR = command;
	while((SPI2->SR & 0x0003) != 0x0003);
	axis = SPI2->DR;
	Delay_us(1);
	SPI2->DR = 0x0000;
	while((SPI2->SR & 0x0003) != 0x0003);
	axis = SPI2->DR;
	GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

	axis >>= 3;

	return axis;
}


void TS_Input(TS_HandleTypeDef *pTSHandle)
{
	uint8_t i;
	uint16_t temp;

	pTSHandle->x_12bit = 0;
	pTSHandle->y_12bit = 0;

	for(i = 0; i < 16; i++)
	{
		if(GPIO_ReadPin(GPIOB, GPIO_PIN_4)  == GPIO_PIN_RESET)
		{
			pTSHandle->x_12bit += TS_Read_ADS7846(pTSHandle->Init.ADS7846_CMD_X);
		}
		else
		{
			pTSHandle->x_12bit = 0;
			pTSHandle->y_12bit = 0;
			break;
		}

		Delay_us(10);

		if(GPIO_ReadPin(GPIOB, GPIO_PIN_4)  == GPIO_PIN_RESET)
		{
			pTSHandle->y_12bit += TS_Read_ADS7846(pTSHandle->Init.ADS7846_CMD_Y);
		}
		else
		{
			pTSHandle->x_12bit = 0;
			pTSHandle->y_12bit = 0;
			break;
		}

		Delay_us(10);
	}

	pTSHandle->x_12bit >>= 4;
	pTSHandle->y_12bit >>= 4;

	if((pTSHandle->x_12bit == 0) && (pTSHandle->y_12bit == 0))
	{
		pTSHandle->x_touch = 0;
		pTSHandle->y_touch = 0;
		return;
	}

	if(pTSHandle->x_12bit <= pTSHandle->Init.x_touch_min)
	{
		pTSHandle->x_touch = 0;
	}
	else if(pTSHandle->x_12bit >= pTSHandle->Init.x_touch_max)
	{
		pTSHandle->x_touch = 239;
	}
	else
	{
		pTSHandle->x_touch = (uint32_t)((float)(pTSHandle->x_12bit - pTSHandle->Init.x_touch_min) * 239./(float)(pTSHandle->Init.x_touch_max - pTSHandle->Init.x_touch_min));
	}

	if(pTSHandle->y_12bit <= pTSHandle->Init.y_touch_min)
	{
		pTSHandle->y_touch = 319;
	}
	else if(pTSHandle->y_12bit >= pTSHandle->Init.y_touch_max)
	{
		pTSHandle->y_touch = 0;
	}
	else
	{
		pTSHandle->y_touch = 319 - (uint32_t)((float)(pTSHandle->y_12bit - pTSHandle->Init.y_touch_min) * 319./(float)(pTSHandle->Init.y_touch_max - pTSHandle->Init.y_touch_min));
	}

	if(TFT1Handle.ScreenMode == 'L')
	{
		temp = pTSHandle->x_12bit;
		pTSHandle->x_12bit = pTSHandle->y_12bit;
		pTSHandle->y_12bit = temp;
		temp = pTSHandle->x_touch;
		pTSHandle->x_touch = pTSHandle->y_touch;
		pTSHandle->y_touch = 239 - temp;
	}
}


__weak void TS_MspInit(TS_HandleTypeDef *pTSHandle)
{
	/* Prevent unused argument(s) compilation warning */
		UNUSED(pTSHandle);

	/* NOTE : This function should not be modified, when the callback is needed,
	 * 		  the TS_MspInit could be implemented in the user file
	 * 		  (This is a weak implementation. The user application may override this function)
	 */
}
