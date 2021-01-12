/*
 * bldc.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Ganghyeok Lim
 */


#include "main.h"
//#include "bldc.h"


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void BLDC_Init(BLDC_HandleTypeDef *pBLDCHandle)
{
	// Init the Low level hardware of BLDC : GPIO, EXTI, TIMER
	BLDC_MspInit(pBLDCHandle);

}


__weak void BLDC_MspInit(BLDC_HandleTypeDef *pBLDCHandle)
{
	/* Prevent unused argument(s) compilation warning */
		UNUSED(pBLDCHandle);

	/* NOTE : This function should not be modified, when the callback is needed,
	 * 		  the pBLDCHandle could be implemented in the user file
	 * 		  (This is a weak implementation. The user application may override this function)
	 */
}


void BLDC_Drive(BLDC_HandleTypeDef *pBLDCHandle)
{
	switch(pBLDCHandle->HallPhase)
	{
		case Phase1:
		{
			if(pBLDCHandle->RotationDir == CW)				BLDC_Step5(pBLDCHandle);
			else if(pBLDCHandle->RotationDir == CCW)		BLDC_Step2(pBLDCHandle);
			break;
		}

		case Phase2:
		{
			if(pBLDCHandle->RotationDir == CW)				BLDC_Step4(pBLDCHandle);
			else if(pBLDCHandle->RotationDir == CCW)		BLDC_Step1(pBLDCHandle);
			break;
		}

		case Phase3:
		{
			if(pBLDCHandle->RotationDir == CW)				BLDC_Step3(pBLDCHandle);
			else if(pBLDCHandle->RotationDir == CCW)		BLDC_Step6(pBLDCHandle);
			break;
		}

		case Phase4:
		{
			if(pBLDCHandle->RotationDir == CW)				BLDC_Step2(pBLDCHandle);
			else if(pBLDCHandle->RotationDir == CCW)		BLDC_Step5(pBLDCHandle);
			break;
		}

		case Phase5:
		{
			if(pBLDCHandle->RotationDir == CW)				BLDC_Step1(pBLDCHandle);
			else if(pBLDCHandle->RotationDir == CCW)		BLDC_Step4(pBLDCHandle);
			break;
		}

		case Phase6:
		{
			if(pBLDCHandle->RotationDir == CW)				BLDC_Step6(pBLDCHandle);
			else if(pBLDCHandle->RotationDir == CCW)		BLDC_Step3(pBLDCHandle);
			break;
		}

		default :
			break;
	}
}


void BLDC_Get_Speed(BLDC_HandleTypeDef *pBLDCHandle, double Ts)
{
	int16_t deltaHallCount;

	deltaHallCount = pBLDCHandle->HallCount - pBLDCHandle->OldHallCount;
	pBLDCHandle->OldHallCount = pBLDCHandle->HallCount;

	pBLDCHandle->CurSpeed = 60. * (double)deltaHallCount / (pBLDCHandle->MotorPoleNum * pBLDCHandle->MotorGearRatio * 3.) / Ts;
}


void BLDC_Get_Position(BLDC_HandleTypeDef *pBLDCHandle)
{
	switch(pBLDCHandle->HallPhase)
	{
		case Phase1:
		{
			if(pBLDCHandle->OldHallPhase == Phase2)				pBLDCHandle->HallCount++;
			else if(pBLDCHandle->OldHallPhase == Phase6)		pBLDCHandle->HallCount--;
			break;
		}
		case Phase2:
		{
			if(pBLDCHandle->OldHallPhase == Phase3)				pBLDCHandle->HallCount++;
			else if(pBLDCHandle->OldHallPhase == Phase1)		pBLDCHandle->HallCount--;
			break;
		}
		case Phase3:
		{
			if(pBLDCHandle->OldHallPhase == Phase4)				pBLDCHandle->HallCount++;
			else if(pBLDCHandle->OldHallPhase == Phase2)		pBLDCHandle->HallCount--;
			break;
		}
		case Phase4:
		{
			if(pBLDCHandle->OldHallPhase == Phase5)				pBLDCHandle->HallCount++;
			else if(pBLDCHandle->OldHallPhase == Phase3)		pBLDCHandle->HallCount--;
			break;
		}
		case Phase5:
		{
			if(pBLDCHandle->OldHallPhase == Phase6)				pBLDCHandle->HallCount++;
			else if(pBLDCHandle->OldHallPhase == Phase4)		pBLDCHandle->HallCount--;
			break;
		}
		case Phase6:
		{
			if(pBLDCHandle->OldHallPhase == Phase1)				pBLDCHandle->HallCount++;
			else if(pBLDCHandle->OldHallPhase == Phase5)		pBLDCHandle->HallCount--;
			break;
		}
		default :
			break;
	}

	pBLDCHandle->CurPosition = (pBLDCHandle->HallCount) * (pBLDCHandle->MotorResolution);

	pBLDCHandle->OldHallPhase = pBLDCHandle->HallPhase;
}


void BLDC_BootstrapCap_Charge(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. Clear GPIO pin of Top side(UT, VT, WT) and Disable All PWM channels
	GPIO_WritePin(pBLDCHandle->Init.GPIOx_Top, pBLDCHandle->Init.GPIO_Pins_Top, GPIO_PIN_RESET);
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_2);
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_3);
	Delay_ms(10);

	// 2. Re-initialize GPIO pins from TIM PWM channels to GPIO Output mode
	GPIO_InitTypeDef GPIOInit;

	memset(&GPIOInit, 0, sizeof(GPIOInit));

	// 3. Re-initialize GPIO pins to GPIO Output mode
	GPIOInit.Pin = pBLDCHandle->Init.GPIO_Pins_Bottom;
	GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOInit.Pull = GPIO_NOPULL;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(pBLDCHandle->Init.GPIOx_Bottom, &GPIOInit);
	Delay_ms(10);

	// 4. Charge Bootstrap Capacitor for 10ms
	GPIO_WritePin(pBLDCHandle->Init.GPIOx_Bottom, pBLDCHandle->Init.GPIO_Pins_Bottom, GPIO_PIN_SET);
	Delay_ms(10);
	GPIO_WritePin(pBLDCHandle->Init.GPIOx_Bottom, pBLDCHandle->Init.GPIO_Pins_Bottom, GPIO_PIN_RESET);

	// 5. Re-initialize GPIO pins from GPIO Output mode to TIM PWM channels
	memset(&GPIOInit, 0, sizeof(GPIOInit));

	GPIOInit.Pin = pBLDCHandle->Init.GPIO_Pins_Bottom;
	GPIOInit.Mode = GPIO_MODE_AF_PP;
	GPIOInit.Pull = GPIO_NOPULL;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(pBLDCHandle->Init.GPIOx_Bottom, &GPIOInit);
	Delay_ms(10);

	// 6. Enable All PWM channels
	TIM_ENABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_1);
	TIM_ENABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_2);
	TIM_ENABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_3);
	Delay_ms(10);
}


void BLDC_Step1(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. UT Logic On (PB0)
	GPIO_ModifyPin(GPIOB, pBLDCHandle->Init.GPIO_Pin_UT, (pBLDCHandle->Init.GPIO_Pin_VT | pBLDCHandle->Init.GPIO_Pin_WT));

	// 2. VB PWM On (PB7)
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_3);
	TIM_ENABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_2);
}


void BLDC_Step2(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. WT Logic On (PB2)
	GPIO_ModifyPin(GPIOB, pBLDCHandle->Init.GPIO_Pin_WT, (pBLDCHandle->Init.GPIO_Pin_UT | pBLDCHandle->Init.GPIO_Pin_VT));

	// 2. VB PWM On (PB7)
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_3);
	TIM_ENABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_2);
}


void BLDC_Step3(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. WT Logic On (PB2)
	GPIO_ModifyPin(GPIOB, pBLDCHandle->Init.GPIO_Pin_WT, (pBLDCHandle->Init.GPIO_Pin_UT | pBLDCHandle->Init.GPIO_Pin_VT));

	// 2. UB PWM On (PB6)
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_2);
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_3);
	TIM_ENABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_1);
}


void BLDC_Step4(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. VT Logic On (PB1)
	GPIO_ModifyPin(GPIOB, pBLDCHandle->Init.GPIO_Pin_VT, (pBLDCHandle->Init.GPIO_Pin_UT | pBLDCHandle->Init.GPIO_Pin_WT));

	// 2. UB PWM On (PB6)
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_2);
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_3);
	TIM_ENABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_1);
}


void BLDC_Step5(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. VT Logic On (PB1)
	GPIO_ModifyPin(GPIOB, pBLDCHandle->Init.GPIO_Pin_VT, (pBLDCHandle->Init.GPIO_Pin_UT | pBLDCHandle->Init.GPIO_Pin_WT));

	// 2. WB PWM On (PB8)
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_2);
	TIM_ENABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_3);
}


void BLDC_Step6(BLDC_HandleTypeDef *pBLDCHandle)
{
	// 1. UT Logic On (PB0)
	GPIO_ModifyPin(GPIOB, pBLDCHandle->Init.GPIO_Pin_UT, (pBLDCHandle->Init.GPIO_Pin_VT | pBLDCHandle->Init.GPIO_Pin_WT));

	// 2. WB PWM On (PB8)
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_2);
	TIM_ENABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_3);
}



/**************************************************************************************************************
 * 																											  *
 * 										   BLDC APIs related to PID control									  *
 * 										   																	  *
 **************************************************************************************************************/

void BLDC_CalculatePID(BLDC_HandleTypeDef *pBLDCHandle, double refValue, double curValue, double dt)
{
	pBLDCHandle->Error = refValue - curValue;

	pBLDCHandle->P_term = pBLDCHandle->Kp * pBLDCHandle->Error;
	pBLDCHandle->I_term += pBLDCHandle->Ki * pBLDCHandle->Error * dt;
	pBLDCHandle->D_term = pBLDCHandle->Kd * (pBLDCHandle->Error - pBLDCHandle->PrvError) / dt;

	pBLDCHandle->PwmPID = pBLDCHandle->P_term + pBLDCHandle->I_term + pBLDCHandle->D_term;

	pBLDCHandle->PrvError = pBLDCHandle->Error;
}



void BLDC_SpeedPID(BLDC_HandleTypeDef *pBLDCHandle, double dt)
{
	/* Get PWM duty cycle which is calculated by Error value and PID gain */
	BLDC_CalculatePID(pBLDCHandle, pBLDCHandle->RefSpeed, pBLDCHandle->CurSpeed, dt);

	/* Figure out Rotation direction */
	if(pBLDCHandle->PwmPID >= 0)		pBLDCHandle->RotationDir = CW;
	else if(pBLDCHandle->PwmPID < 0)	pBLDCHandle->RotationDir = CCW;

	/* Saturate PWM duty if it exceeds the limit of PWM duty value */
	uint16_t PwmPID_ABS = (uint16_t)(abs(pBLDCHandle->PwmPID));

	if(PwmPID_ABS > 95)		PwmPID_ABS = 95;

	SetPwmDuty(pBLDCHandle, PwmPID_ABS);
}


void BLDC_PositionPID(BLDC_HandleTypeDef *pBLDCHandle, double dt)
{
	/* Calculate Trajectory Position for given Reference position, Current position, Vmax, Acceleration values  */
	//BLDC_CalculateTrajectoryPosition(pBLDCHandle, dt);

	/* Get PWM duty cycle which is calculated by Error value and PID gain */
	BLDC_CalculatePID(pBLDCHandle, pBLDCHandle->RefPosition, pBLDCHandle->CurPosition, dt);

	static double pidLimit = 3600.;

	/* Limit the PID control output */
	if(pBLDCHandle->PwmPID > pidLimit)					pBLDCHandle->PwmPID = pidLimit;
	else if(pBLDCHandle->PwmPID < (-1)*pidLimit)		pBLDCHandle->PwmPID = (-1)*pidLimit;

	if(pBLDCHandle->PwmPID >= 0)
	{
		pBLDCHandle->RotationDir = CW;

		SetPwmDuty(pBLDCHandle, (uint16_t)(pBLDCHandle->PwmPID));	// 5% 여유 필요한가
	}
	else if(pBLDCHandle->PwmPID < 0)
	{
		pBLDCHandle->RotationDir = CCW;

		SetPwmDuty(pBLDCHandle, (uint16_t)((-1) * pBLDCHandle->PwmPID));
	}
}


void BLDC_CalculateTrajectoryPosition(BLDC_HandleTypeDef *pBLDCHandle, double dt)
{
	double deltaRefPos;
	double accelIntervalPos;
	double remainingPos;
	int rotationDir;
	int accelDir;

	/* Calculate the Total Reference Position value to Move */
	deltaRefPos = pBLDCHandle->RefPosition - pBLDCHandle->PrvRefPosition;

	/* Calculate the Position value to Move in Acceleration interval */
	accelIntervalPos = (0.5) * (pBLDCHandle->TrjRefMaxSpeed * pBLDCHandle->TrjRefMaxSpeed) / pBLDCHandle->TrjRefAcceleration;

	/* Calculate the Remaining Position value */
	remainingPos = pBLDCHandle->RefPosition - pBLDCHandle->CurPosition;

	/* Figure out the Direction to Rotate */
	if(remainingPos >= 0)			rotationDir = CW;
	else if(remainingPos < 0)		rotationDir = CCW;


	/* When the Speed profile is Trapezoidal */
	if( accelIntervalPos < (0.5 * deltaRefPos) )
	{
		if( (pBLDCHandle->CurPosition - pBLDCHandle->PrvRefPosition) < accelIntervalPos )			accelDir = 1;		// Motor is in Acceleration interval
		else if( remainingPos > accelIntervalPos )													accelDir = 0;		// Motor is in Constant Velocity interval
		else																						accelDir = -1;		// Motor is in Deceleration interval
	}
	/* When the Speed profile is Isosceles triangle */
	else if( accelIntervalPos >= (0.5 * deltaRefPos) )
	{
		if( (pBLDCHandle->CurPosition - pBLDCHandle->PrvRefPosition) < (0.5 * deltaRefPos) )		accelDir = 1;
		else																						accelDir = -1;
	}


	switch (accelDir)
	{
		case 1:
		{
			pBLDCHandle->TrjDtAcceleration = pBLDCHandle->TrjRefAcceleration * dt;
			break;
		}

		case 0:
		{
			pBLDCHandle->TrjDtAcceleration = 0;
			break;
		}

		case -1:
		{
			pBLDCHandle->TrjDtAcceleration = (-1) * pBLDCHandle->TrjRefAcceleration * dt;
			break;
		}

		default :
			break;
	}


	pBLDCHandle->TrjCurSpeed += pBLDCHandle->TrjDtAcceleration;


	switch (rotationDir)
	{
		case CW:
		{
			if(pBLDCHandle->TrjCurPosition >= pBLDCHandle->RefPosition)
			{
				pBLDCHandle->TrjCurSpeed = 0;
				pBLDCHandle->TrjDtAcceleration = 0;
				pBLDCHandle->PrvRefPosition = pBLDCHandle->RefPosition;
			}
			else
			{
				pBLDCHandle->TrjCurPosition += (0.5) * dt * ((2 * pBLDCHandle->TrjCurSpeed) - pBLDCHandle->TrjDtAcceleration);
			}

			break;
		}

		case CCW:
		{
			if(pBLDCHandle->TrjCurPosition <= pBLDCHandle->RefPosition)
			{
				pBLDCHandle->TrjCurSpeed = 0;
				pBLDCHandle->TrjDtAcceleration = 0;
				pBLDCHandle->PrvRefPosition = pBLDCHandle->RefPosition;
			}
			else
			{
				pBLDCHandle->TrjCurPosition += (0.5) * dt * ((2 * pBLDCHandle->TrjCurSpeed) - pBLDCHandle->TrjDtAcceleration);
			}

			break;
		}

		default :
			break;
	}
}



/**************************************************************************************************************
 * 																											  *
 * 										  	   BLDC Test functions			 								  *
 * 										   																	  *
 **************************************************************************************************************/

void IR2101_Test1(uint16_t Top_time_us, uint16_t Low_time_us, uint16_t Dead_time_us)
{
	GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	Delay_us(Low_time_us);
	GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	Delay_us(Dead_time_us);
	GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	Delay_us(Top_time_us);
	GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	Delay_us(Dead_time_us);
}

void IR2101_Test2(uint16_t Top_time_ms, uint16_t Low_time_ms, uint16_t Dead_time_ms)
{
	GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	Delay_ms(Low_time_ms);
	GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	Delay_ms(Dead_time_ms);
	GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	Delay_ms(Top_time_ms);
	GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	Delay_ms(Dead_time_ms);
}

void IR2101_Test3(TIM_HandleTypeDef *pTIMHandle, uint16_t Top_time_us, uint16_t Low_time_us, uint16_t Dead_time_us)
{
	TIM_SET_COMPARE(pTIMHandle, TIM_CHANNEL_1, 50);
	Delay_us(Low_time_us);
	TIM_SET_COMPARE(pTIMHandle, TIM_CHANNEL_1, 0);
	Delay_us(Dead_time_us);
	GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	Delay_us(Top_time_us);
	GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	Delay_us(Dead_time_us);
}

void BootCharge_Test(uint16_t tim_on_time_us, uint16_t tim_off_time_us)
{
	GPIO_HandleTypeDef GPIOHandle_TIM1;
	memset(&GPIOHandle_TIM1, 0, sizeof(GPIOHandle_TIM1));

	// 1. Enable TIM1 & Config port for TIM1
	GPIOHandle_TIM1.Instance = GPIOA;
	GPIOHandle_TIM1.Init.Pin = GPIO_PIN_8;
	GPIOHandle_TIM1.Init.Mode = GPIO_MODE_AF_PP;
	GPIOHandle_TIM1.Init.Pull = GPIO_NOPULL;
	GPIOHandle_TIM1.Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(GPIOHandle_TIM1.Instance, &GPIOHandle_TIM1.Init);
	SET_BIT(TIM1->CR1, TIM_CR1_CEN);
	SET_BIT(TIM1->CCER, TIM_CCER_CC1E);


	Delay_us(tim_on_time_us);

	// 2. Disable TIM1 & Config port for GPIO Output mode
	CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);
	CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1E);
	GPIOHandle_TIM1.Instance = GPIOA;
	GPIOHandle_TIM1.Init.Pin = GPIO_PIN_8;
	GPIOHandle_TIM1.Init.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOHandle_TIM1.Init.Pull = GPIO_NOPULL;
	GPIOHandle_TIM1.Init.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_Init(GPIOHandle_TIM1.Instance, &GPIOHandle_TIM1.Init);
	GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	Delay_us(tim_off_time_us);
}
