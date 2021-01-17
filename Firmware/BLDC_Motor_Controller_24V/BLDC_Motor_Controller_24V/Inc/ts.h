/*
 * ts.h
 *
 *  Created on: Jan 16, 2021
 *      Author: Ganghyeok Lim
 */

#ifndef TS_H_
#define TS_H_

#include "stm32f103xx.h"
#include "common.h"


/* Touch Screen type */
#define TS1						1
#define TS2						2

/*=============================================================================================================================*/

/**
  * @brief  Touch Screen Configuration Structure definition
  */
typedef struct
{
	GPIO_TypeDef			*GPIOx_TS_Control;

	uint32_t				GPIO_Pin_TS_nCS;

	uint32_t				GPIO_Pin_TS_nINT;

	uint32_t				x_touch_min;

	uint32_t				x_touch_max;

	uint32_t				y_touch_min;

	uint32_t				y_touch_max;

	uint16_t				ADS7846_CMD_X;

	uint16_t				ADS7846_CMD_Y;

	SPI_HandleTypeDef 		*SPI_Handle;

} TS_InitTypeDef;


/**
  * @brief  Touch Screen Handle Structure definition
  */
typedef struct
{
	uint32_t				Instance;

	TS_InitTypeDef			Init;

	uint16_t				x_12bit;

	uint16_t				y_12bit;

	uint16_t				x_touch;

	uint16_t				y_touch;

} TS_HandleTypeDef;


/**************************************************************************************************************
 * 																											  *
 * 												User Macro Definition										  *
 * 									  																		  *
 **************************************************************************************************************/





/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void TS_Init(TS_HandleTypeDef *pTSHandle);
void TS_MspInit(TS_HandleTypeDef *pTSHandle);
uint16_t TS_Read_ADS7846(uint16_t command);
void TS_Input(TS_HandleTypeDef *pTSHandle);

#endif /* TS_H_ */
