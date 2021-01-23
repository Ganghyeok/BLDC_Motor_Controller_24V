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


/* BLDC Motor type */
#define BLDC1									1
#define BLDC2									2


/* BLDC Macro definitions */
#define Phase1									3
#define Phase2									2
#define Phase3									6
#define Phase4									4
#define Phase5									5
#define Phase6									1
#define MOTOR_STATE_STOP						0
#define MOTOR_STATE_SPEED						1
#define MOTOR_STATE_POSITION					2
#define MOTOR_STATE_POSITION_TRACKING			3
#define CW										0
#define CCW										1

/* BLDC Macro functions */
#define BLDC_SET_ROTATION_DIRECTION(_HANDLE_, dir)		( (_HANDLE_)->RotationDir = (dir == CW) ? CW : CCW )

#define BLDC_SET_REFERENCE_DUTY(duty)			( DutyRef = (duty > 95) ? 95 : \
															(duty < 0)  ? 0  : duty )

#define BLDC_SET_REFERENCE_SPEED(_HANDLE_, speed)			( (_HANDLE_)->RefSpeed = speed )

#define BLDC_SET_REFERENCE_POSITION(_HANDLE_, position)		( (_HANDLE_)->RefPosition = position )

#define BLDC_SET_OLD_HALLPHASE(_HANDLE_)		do{																								\
																																				\
														(_HANDLE_)->HallPhase = (READ_BIT(GPIOC->IDR, (_HANDLE_)->Init.GPIO_Pins_Hall)) >> 6U;	\
																																				\
														switch( (_HANDLE_)->HallPhase )															\
														{																						\
															case Phase1:																		\
															{																					\
																if((_HANDLE_)->RotationDir == CW)			(_HANDLE_)->OldHallPhase = Phase2;	\
																else if((_HANDLE_)->RotationDir == CCW)		(_HANDLE_)->OldHallPhase = Phase6;	\
																break;																			\
															}																					\
															case Phase2:																		\
															{																					\
																if((_HANDLE_)->RotationDir == CW)			(_HANDLE_)->OldHallPhase = Phase3;	\
																else if((_HANDLE_)->RotationDir == CCW)		(_HANDLE_)->OldHallPhase = Phase1;	\
																break;																			\
															}																					\
															case Phase3:																		\
															{																					\
																if((_HANDLE_)->RotationDir == CW)			(_HANDLE_)->OldHallPhase = Phase4;	\
																else if((_HANDLE_)->RotationDir == CCW)		(_HANDLE_)->OldHallPhase = Phase2;	\
																break;																			\
															}																					\
															case Phase4:																		\
															{																					\
																if((_HANDLE_)->RotationDir == CW)			(_HANDLE_)->OldHallPhase = Phase5;	\
																else if((_HANDLE_)->RotationDir == CCW)		(_HANDLE_)->OldHallPhase = Phase3;	\
																break;																			\
															}																					\
															case Phase5:																		\
															{																					\
																if((_HANDLE_)->RotationDir == CW)			(_HANDLE_)->OldHallPhase = Phase6;	\
																else if((_HANDLE_)->RotationDir == CCW)		(_HANDLE_)->OldHallPhase = Phase4;	\
																break;																			\
															}																					\
															case Phase6:																		\
															{																					\
																if((_HANDLE_)->RotationDir == CW)			(_HANDLE_)->OldHallPhase = Phase1;	\
																else if((_HANDLE_)->RotationDir == CCW)		(_HANDLE_)->OldHallPhase = Phase5;	\
																break;																			\
															}																					\
															default :																			\
																break;																			\
														}																						\
																																				\
																																				\
													}while(0)


#define BLDC_PID_GAIN_SET(_HANDLE_, P_Gain, I_Gain, D_Gain)		do{									\
																		(_HANDLE_)->Kp = P_Gain;	\
																		(_HANDLE_)->Ki = I_Gain;	\
																		(_HANDLE_)->Kd = D_Gain;	\
																	}while(0)


/* BLDC global variables */




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

	TIM_HandleTypeDef 	*TIM_Handle;

} BLDC_InitTypeDef;


typedef struct
{
	/* Motor Hardware related Parameter */
	uint8_t					Instance;

	BLDC_InitTypeDef		Init;

	uint8_t					MotorPoleNum;

	uint8_t					MotorGearRatio;

	double					MotorResolution;

	/* Motor Control related Parameter*/
	uint8_t 				MotorState;

	uint8_t 				RotationDir;

	uint16_t 				HallPhase;

	uint16_t 				OldHallPhase;

	int32_t 				HallCount;

	int32_t					OldHallCount;

	double					CurSpeed;

	double					RefSpeed;				// User configurable parameter

	double 					CurPosition;

	double					RefPosition;			// User configurable parameter

	double					PrvRefPosition;

	/* Motor Position Trajectory related Parameter */
	double					TrjCurPosition;

	double					TrjCurSpeed;

	double					TrjRefMaxSpeed;			// User configurable parameter

	double					TrjRefAcceleration;		// User configurable parameter

	double					TrjDtAcceleration;

	/* Motor PID Control related Parameter */
	double					Kp;

	double					Ki;

	double					Kd;

	double					Error;

	double					PrvError;

	double					P_term;

	double					I_term;

	double					D_term;

	double					PwmPID;

} BLDC_HandleTypeDef;



/* BLDC Motor functions */
void BLDC_Init(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_MspInit(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Drive(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Get_Speed(BLDC_HandleTypeDef *pBLDCHandle, double Ts);
void BLDC_Get_Position(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_BootstrapCap_Charge(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Step1(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Step2(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Step3(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Step4(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Step5(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_Step6(BLDC_HandleTypeDef *pBLDCHandle);
void BLDC_CalculatePID(BLDC_HandleTypeDef *pBLDCHandle, double refValue, double curValue, double dt);
void BLDC_SpeedPID(BLDC_HandleTypeDef *pBLDCHandle, double dt);
void BLDC_PositionPID(BLDC_HandleTypeDef *pBLDCHandle, double dt);
void BLDC_CalculateTrajectoryPosition(BLDC_HandleTypeDef *pBLDCHandle, double dt);

void IR2101_Test1(uint16_t Top_time_us, uint16_t Low_time_us, uint16_t Dead_time_us);
void IR2101_Test2(uint16_t Top_time_ms, uint16_t Low_time_ms, uint16_t Dead_time_ms);
void IR2101_Test3(TIM_HandleTypeDef *pTIMHandle, uint16_t Top_time_us, uint16_t Low_time_us, uint16_t Dead_time_us);
void BootCharge_Test(uint16_t tim_on_time_us, uint16_t tim_off_time_us);





#endif /* BLDC_H_ */
