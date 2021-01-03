/*
 * bldc.h
 *
 *  Created on: 2021. 1. 3.
 *      Author: Ganghyeok Lim
 */

#ifndef BLDC_H_
#define BLDC_H_

#include "stm32f103xx.h"
#include "common.h"


/* BLDC Macro definitions */
#define Phase1			3
#define Phase2			2
#define Phase3			6
#define Phase4			4
#define Phase5			5
#define Phase6			1
#define START			0
#define STOP			1


/* BLDC Macro functions */
#define BLDC_SET_REF_DUTY(duty)			(DutyRef = duty)


/* BLDC Configuration structure */

typedef struct
{
	GPIO_TypeDef 		*GPIOx_Top;

	GPIO_TypeDef 		*GPIOx_Bottom;

	GPIO_TypeDef 		*GPIOx_Hall;

	uint32_t 			GPIO_Pins_Top;

	uint32_t 			GPIO_Pins_Bottom;

	uint32_t 			GPIO_Pins_Hall;

	uint32_t			GPIO_Pin_UT;

	uint32_t			GPIO_Pin_VT;

	uint32_t			GPIO_Pin_WT;

	uint32_t			GPIO_Pin_UB;

	uint32_t			GPIO_Pin_VB;

	uint32_t			GPIO_Pin_WB;

	uint32_t			GPIO_Pin_HA;

	uint32_t			GPIO_Pin_HB;

	uint32_t			GPIO_Pin_HC;

} BLDC_GPIOList;


typedef struct
{
	BLDC_GPIOList		GPIO_List;

	TIM_HandleTypeDef 	*TIM_Handle;

} BLDC_HandleTypeDef;


/* BLDC Motor functions */
void BLDC_Drive(BLDC_HandleTypeDef *pBLDCHandle, uint16_t hallPhase);
void BLDC_BootstrapCap_Charge(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Step1(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Step2(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Step3(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Step4(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Step5(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Step6(BLDC_HandleTypeDef *pBLDCHandle);

void IR2101_Test1(uint16_t Top_time_us, uint16_t Low_time_us, uint16_t Dead_time_us);
void IR2101_Test2(uint16_t Top_time_ms, uint16_t Low_time_ms, uint16_t Dead_time_ms);
void IR2101_Test3(TIM_HandleTypeDef *pTIMHandle, uint16_t Top_time_us, uint16_t Low_time_us, uint16_t Dead_time_us);
void BootCharge_Test(uint16_t tim_on_time_us, uint16_t tim_off_time_us);





#endif /* BLDC_H_ */
