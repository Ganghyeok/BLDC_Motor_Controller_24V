/*
 * user_func.c
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#include "main.h"


TIM_HandleTypeDef 		TIM6Handle;
TIM_HandleTypeDef 		TIM4Handle;
BLDC_HandleTypeDef 		BLDC1Handle;
UART_HandleTypeDef 		UART3Handle;
DMA_HandleTypeDef		DMA1Handle;
TFT_HandleTypeDef		TFT1Handle;
TS_HandleTypeDef		TS1Handle;
SPI_HandleTypeDef		SPI2Handle;


uint8_t ButtonFlag = FLAG_RESET;


char MotorSpeedStr[6] = {0,};
char MotorPositionStr[8] = {0,};
char PwmPidStr[5] = {0,};
char PwmPidAbsStr[4] = {0,};
char Msg1[50] = {0,};

uint8_t startFlag = FLAG_RESET;


/********************************************************************************************************************
 * 																											  		*
 *											Application Specific Function											*
 * 																											  		*
 ********************************************************************************************************************/

/********************************************************************************************************************
 *												Initialization Function												*
 ********************************************************************************************************************/

void Button_Init(void)
{
	GPIO_InitTypeDef GPIOInit;

	memset(&GPIOInit, 0, sizeof(GPIOInit));

	// 1. Initialize GPIO for START/STOP Button
	GPIOInit.Pin = GPIO_PIN_4;
	GPIOInit.Mode = GPIO_MODE_INPUT;
	GPIOInit.Pull = GPIO_PULLUP;
	GPIO_Init(GPIOA, &GPIOInit);
}


void BLDC1_Init(void)
{
	/* Initialize Motor Hardware related Parameter */
	BLDC1Handle.Instance = BLDC1;
	BLDC1Handle.MotorPoleNum = 8;
	BLDC1Handle.MotorGearRatio = 4;
	BLDC1Handle.MotorResolution = (double)360/6/(BLDC1Handle.MotorPoleNum/2)/4;

	/* Initialize Motor Control related Parameter*/
	BLDC1Handle.MotorState = MOTOR_STATE_STOP;
	BLDC1Handle.HallCount = 0;
	BLDC1Handle.OldHallCount = 0;
	BLDC1Handle.CurSpeed = 0;
	BLDC1Handle.RefSpeed = 0;
	BLDC1Handle.CurPosition = 0;
	BLDC1Handle.RefPosition = 0;
	BLDC1Handle.PrvRefPosition = 0;

	/* Initialize Motor Position Trajectory related Parameter */
	BLDC1Handle.TrjCurPosition = 0;
	BLDC1Handle.TrjCurSpeed = 0;
	BLDC1Handle.TrjRefMaxSpeed = 0;
	BLDC1Handle.TrjRefAcceleration = 0;
	BLDC1Handle.TrjDtAcceleration = 0;

	/* Initialize Motor PID Control related Parameter */
	BLDC1Handle.Kp = 0;
	BLDC1Handle.Ki = 0;
	BLDC1Handle.Kd = 0;
	BLDC1Handle.Error = 0;
	BLDC1Handle.PrvError = 0;
	BLDC1Handle.P_term = 0;
	BLDC1Handle.I_term = 0;
	BLDC1Handle.D_term = 0;
	BLDC1Handle.PwmPID = 0;

	BLDC_Init(&BLDC1Handle);
}


void UART3_Init(void)
{
	UART3Handle.Instance = USART3;
	UART3Handle.Init.Mode = UART_MODE_TX;
	UART3Handle.Init.OverSampling = UART_OVERSAMPLING_16;
	UART3Handle.Init.BaudRate = USART_STD_BAUD_115200;
	UART3Handle.Init.Parity = UART_PARITY_NONE;
	UART3Handle.Init.StopBits = UART_STOPBITS_1;
	UART3Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UART3Handle.Init.WordLength = UART_WORDLENGTH_8B;
	UART3Handle.hdmatx = &DMA1Handle;

	USART_Init(&UART3Handle);
}


void TIM6_Init(void)
{
	// Init TIM6 Base
	TIM6Handle.Instance = TIM6;
	TIM6Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM6Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM6Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	TIM6Handle.Init.Prescaler = (7200-1);	// 72MHz / 7200 = 10kHz
	TIM6Handle.Init.Period = (10-1);	// 10kHz / 10 = 1kHz
	TIM6Handle.Init.RepetitionCounter = 0;
	TIM_Base_Init(&TIM6Handle);

	// Enable TIM6 interrupt for Update Event
	TIM_ENABLE_IT(&TIM6Handle, TIM_IT_UPDATE);

	// Enable TIM6 Counter
	TIM_ENABLE_COUNTER(&TIM6Handle);
}


void DMA1_Init(void)
{
	// 1. Enable the peripheral clock for the DMA1
	RCC_DMA1_CLK_ENABLE();

	// 2. Configure the NVIC of DMA1 channel7
	NVIC_IRQConfig(IRQ_NO_DMA1_CHANNEL2, NVIC_PRIOR_15, ENABLE);
}


void DMA1_Interrupt_Configuration(void)
{
	// 1. Enable Half-transfer interrupt
	//DMA1_Channel2->CCR |= (0x1 << 2);

	// 2. Enable Transfer complete interrupt
	DMA1_Channel2->CCR |= (0x1 << 1);

	// 3. Enable Transfer error interrupt
	DMA1_Channel2->CCR |= (0x1 << 3);

	NVIC_IRQConfig(IRQ_NO_DMA1_CHANNEL2, NVIC_PRIOR_15, ENABLE);
}


void TFT1_Init(void)
{
	TFT1Handle.Instance = TFT1;
	TFT1Handle.ScreenMode = 'L';
	TFT1Handle.XcharacterLimit = 40;
	TFT1Handle.YcharacterLimit = 30;
	TFT1Handle.cursor_flag = 0;
	TFT1Handle.underscore_flag = 0;
	TFT1Handle.outline_flag = 0;
	TFT1Handle.Kfont_type = 'M';

	TFT_Init(&TFT1Handle);
}


void TS1_Init(void)
{
	TS1Handle.Instance = TS1;
	TS1Handle.x_12bit = 0;
	TS1Handle.y_12bit = 0;
	TS1Handle.x_touch = 0;
	TS1Handle.y_touch = 0;
	TS1Handle.Init.x_touch_min = 250;
	TS1Handle.Init.x_touch_max = 3700;
	TS1Handle.Init.y_touch_min = 350;
	TS1Handle.Init.y_touch_max = 3750;
	TS1Handle.Init.ADS7846_CMD_X = 0x00D0;
	TS1Handle.Init.ADS7846_CMD_Y = 0x0090;

	TS_Init(&TS1Handle);
}


void Test_Init(void)
{
	GPIO_InitTypeDef DebugLed;

	memset(&DebugLed,0, sizeof(DebugLed));

	DebugLed.Pin = GPIO_PIN_0;
	DebugLed.Mode = GPIO_MODE_OUTPUT_PP;
	DebugLed.Pull = GPIO_NOPULL;
	DebugLed.Speed = GPIO_SPEED_FREQ_MEDIUM;

	GPIO_Init(GPIOA, &DebugLed);
}


/********************************************************************************************************************
 *												  Callback Function													*
 ********************************************************************************************************************/

void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *pTIMHandle)
{
	/* This Callback function is executed every 1ms by TIM6 */

	static int count = 0;
	char sign;


	/* Check the Button is pressed */
	if(ButtonFlag == FLAG_RESET)
	{
		uint8_t buttonState;

		buttonState = READ_BIT(GPIOA->IDR, GPIO_PIN_4);

		if(buttonState == BUTTON_PRESSED)
		{
			ButtonFlag = FLAG_SET;
		}
	}

	/* TIM6 */
	if(pTIMHandle->Instance == TIM6)
	{
		/* Motor State is SPEED */
		if(BLDC1Handle.MotorState == MOTOR_STATE_SPEED)
		{
			if(count >= 100)
			{
				/* Calculate the Current Speed of BLDC Motor */
				BLDC_Get_Speed(&BLDC1Handle, 0.1);

				/* Set PWM duty cycle by Speed PID calculation */
				BLDC_SpeedPID(&BLDC1Handle, 0.1);

				/* Transmit Motor Speed value to PC through UART3 */
				int16_t motorSpeed, motorSpeedAbs;

				motorSpeed = (int16_t)BLDC1Handle.CurSpeed;
				motorSpeedAbs = abs(motorSpeed);

				if(motorSpeed >= 0)			sign = '+';
				else if(motorSpeed < 0)		sign = '-';

				MotorSpeedStr[0] = sign;
				MotorSpeedStr[1] = (motorSpeedAbs / 1000) + 48;
				MotorSpeedStr[2] = ((motorSpeedAbs % 1000) / 100) + 48;
				MotorSpeedStr[3] = ((motorSpeedAbs % 100) / 10) + 48;
				MotorSpeedStr[4] = (motorSpeedAbs % 10) + 48;
				MotorSpeedStr[5] = '\n';

				UART_Transmit_DMA(&UART3Handle, (uint8_t*)MotorSpeedStr, strlen((char*)MotorSpeedStr));

				count = 0;
			}
		}

		/* Motor State is POSITION */
		else if(BLDC1Handle.MotorState == MOTOR_STATE_POSITION)
		{
			/* Set PWM duty cycle by Position PID calculation */
			BLDC_PositionPID(&BLDC1Handle, 0.001);

			startFlag = FLAG_SET;
		}



		/* Transmit Motor Position value to PC through UART3 */
		if(count >= 2)		// Every 2ms
		{
			if(BLDC1Handle.RotationDir == CW)			sign = '+';
			else if(BLDC1Handle.RotationDir == CCW)		sign = '-';

			//sprintf(Msg1, "%.2lf, %.2lf\n", BLDC1Handle.CurPosition, BLDC1Handle.PwmPID);	// To see the case of RefPosition
			sprintf(Msg1, "%.2lf,%.2lf\n", BLDC1Handle.TrjCurPosition, BLDC1Handle.CurPosition);	// To see the case of TrjCurPosition

			UART_Transmit_DMA(&UART3Handle, (uint8_t*)Msg1, strlen((char*)Msg1));

			count = 0;
		}

		count++;
	}
}


void EXTI_Callback(uint32_t GPIO_Pin)
{
	// 1. Detect current HallPhase location
	BLDC1Handle.HallPhase = (READ_BIT(GPIOA->IDR, BLDC1Handle.Init.GPIO_Pins_Hall)) >> 5U;

	// 2. Get current position value
	BLDC_Get_Position(&BLDC1Handle);

	// 3. Drive BLDC motor according to HallPhase location
	BLDC_Drive(&BLDC1Handle);


	UNUSED(GPIO_Pin);
}



/********************************************************************************************************************
 *							Group of functions which belong to main function for increasing Readability				*
 ********************************************************************************************************************/

void MemsetHandleStructure(void)
{
	memset(&TIM6Handle, 0, sizeof(TIM6Handle));
	memset(&TIM4Handle, 0, sizeof(TIM4Handle));
	memset(&BLDC1Handle, 0, sizeof(BLDC1Handle));
	memset(&UART3Handle, 0, sizeof(UART3Handle));
	memset(&DMA1Handle, 0, sizeof(DMA1Handle));
	memset(&TFT1Handle, 0, sizeof(TFT1Handle));
	memset(&TS1Handle, 0, sizeof(TS1Handle));
	memset(&SPI2Handle, 0, sizeof(SPI2Handle));
}


void StartTimerPwm(BLDC_HandleTypeDef *pBLDCHandle)
{
	TIM_PWM_Start(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_1);			// Start PWM for UB
	TIM_PWM_Start(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_2);			// Start PWM for VB
	TIM_PWM_Start(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_3);			// Start PWM for WB
}


void EnableTimerPwmChannel(BLDC_HandleTypeDef *pBLDCHandle)
{
	TIM_ENABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_1);
	TIM_ENABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_2);
	TIM_ENABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_3);
}


void DisableTimerPwmChannel(BLDC_HandleTypeDef *pBLDCHandle)
{
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_1);
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_2);
	TIM_DISABLE_CHANNEL(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_3);
}


void SetPwmDuty(BLDC_HandleTypeDef *pBLDCHandle, uint32_t duty)
{
	TIM_SET_COMPARE(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_1, (uint16_t)duty);
	TIM_SET_COMPARE(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_2, (uint16_t)duty);
	TIM_SET_COMPARE(pBLDCHandle->Init.TIM_Handle, TIM_CHANNEL_3, (uint16_t)duty);
}

