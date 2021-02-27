/*
 * user_func.c
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#include "main.h"


/* Peripheral Handle Definitions */
TIM_HandleTypeDef 					TIM6Handle;
TIM_HandleTypeDef 					TIM4Handle;
BLDC_HandleTypeDef 					BLDC1Handle;
UART_HandleTypeDef 					UART3Handle;
DMA_HandleTypeDef					DMA1Handle;
TFT_HandleTypeDef					TFT1Handle;
TS_HandleTypeDef					TS1Handle;
SPI_HandleTypeDef					SPI2Handle;

/* Status Flags */
uint8_t State = 					STATE_MENU;
uint8_t State_option = 				STATE_SPEED;
uint8_t Recharge_flag = 			FLAG_RESET;

/* Key Variables */
int32_t Mode_key = 					0;			// Count of 'MODE' Key
int32_t Up_key = 					0;			// Count of 'UP' Key
int32_t Down_key = 					0;			// Count of 'DOWN' Key
int32_t Start_key = 				0;			// Count of 'START/STOP' Key

uint8_t KeyFlag_Mode = 				FLAG_RESET;
uint8_t KeyFlag_Up = 				FLAG_RESET;
uint8_t KeyFlag_Down = 				FLAG_RESET;
uint8_t KeyFlag_Start =				FLAG_RESET;

uint32_t KeyTime_Mode = 			0;
uint32_t KeyTime_Up =				0;
uint32_t KeyTime_Down =				0;
uint32_t KeyTime_Start =			0;

/* Touch Screen Variables */
uint8_t TouchDetection_flag = 		FLAG_RESET;
uint32_t TouchTime =				0;
uint16_t xTouch_log = 				0;
uint16_t yTouch_log = 				0;

/* Strings for UART */
char MotorSpeedStr[6] = 			{0,};
char MotorPositionStr[8] = 			{0,};
char Msg1[50] = 					{0,};

/* Graph variables */
uint16_t x = 						0;
uint16_t y = 						0;
uint16_t x_prv = 					0;
uint16_t y_prv = 					0;
uint8_t GraphDraw_flag = 			FLAG_RESET;
uint8_t GraphClear_flag = 			FLAG_RESET;



/********************************************************************************************************************
 * 																											  		*
 *											Application Specific Function											*
 * 																											  		*
 ********************************************************************************************************************/

/********************************************************************************************************************
 *												Initialization Function												*
 ********************************************************************************************************************/

void Key_Init(void)
{
	GPIO_InitTypeDef GPIOInit;

	memset(&GPIOInit, 0, sizeof(GPIOInit));

	/* Init GPIO of MODE Button */
	GPIOInit.Pin = GPIO_PIN_0;
	GPIOInit.Mode = GPIO_MODE_IT_FALLING;
	GPIOInit.Pull = GPIO_PULLUP;
	GPIO_Init(GPIOA, &GPIOInit);
	NVIC_IRQConfig(IRQ_NO_EXTI0, NVIC_PRIOR_15, ENABLE);

	/* Init GPIO of Up Button */
	GPIOInit.Pin = GPIO_PIN_1;
	GPIOInit.Mode = GPIO_MODE_IT_FALLING;
	GPIOInit.Pull = GPIO_PULLUP;
	GPIO_Init(GPIOA, &GPIOInit);
	NVIC_IRQConfig(IRQ_NO_EXTI1, NVIC_PRIOR_15, ENABLE);

	/* Init GPIO of Down Button */
	GPIOInit.Pin = GPIO_PIN_2;
	GPIOInit.Mode = GPIO_MODE_IT_FALLING;
	GPIOInit.Pull = GPIO_PULLUP;
	GPIO_Init(GPIOA, &GPIOInit);
	NVIC_IRQConfig(IRQ_NO_EXTI2, NVIC_PRIOR_15, ENABLE);

	/* Init GPIO of START/STOP Button */
	GPIOInit.Pin = GPIO_PIN_3;
	GPIOInit.Mode = GPIO_MODE_IT_FALLING;
	GPIOInit.Pull = GPIO_PULLUP;
	GPIO_Init(GPIOA, &GPIOInit);
	NVIC_IRQConfig(IRQ_NO_EXTI3, NVIC_PRIOR_15, ENABLE);
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


void TFT1_Init(void)
{
	TFT1Handle.Instance = TFT1;
	TFT1Handle.ScreenMode = 'L';
	TFT1Handle.XcharacterLimit = 40;
	TFT1Handle.YcharacterLimit = 30;
	TFT1Handle.XcharacterLimit_Large = 26;
	TFT1Handle.YcharacterLimit_Large = 30;
	TFT1Handle.nextline_flag = 0;
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

	SPI_ENABLE(&SPI2Handle);
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
	// This Callback function is executed every 1ms by TIM6
	static int count0 = 0;
	static int hallCnt20ms = 0;
	static int hallCnt20msOld = 0;;
	static int diffHallCnt20ms;

	/* TIM6 */
	if(pTIMHandle->Instance == TIM6)
	{
		// Detect Key input
		Detect_KeyInput();


		// Detect Touch Screen input
		Detect_TouchScreenInput();


		// If Motor State is SPEED
		if(BLDC1Handle.MotorState == MOTOR_STATE_SPEED)
		{
			BLDC_SpeedMode();


			if(count0 >= 20)
			{
				if(Recharge_flag == FLAG_RESET)
				{
					hallCnt20ms = BLDC1Handle.HallCount;
					diffHallCnt20ms = hallCnt20ms - hallCnt20msOld;
					hallCnt20msOld = hallCnt20ms;

					if(diffHallCnt20ms == 0)
					{
						Recharge_flag = FLAG_SET;
					}
				}

				count0 = 0;
			}

			count0++;
		}


		// If Motor State is POSITION or POSITION_TRACKING
		else if( (BLDC1Handle.MotorState == MOTOR_STATE_POSITION) || (BLDC1Handle.MotorState == MOTOR_STATE_POSITION_TRACKING) )
		{
			BLDC_PositionMode();
		}
	}
}


void BLDC_SpeedMode(void)
{
	static int count1 = 0;


	if(count1 >= 100)
	{
		// 1. Calculate the Current Speed of BLDC Motor
		BLDC_Get_Speed(&BLDC1Handle, 0.1);

		// 2. Set PWM duty cycle by Speed PID calculation
		BLDC_SpeedPID(&BLDC1Handle, 0.1);


		// 3. Calculate the Pixel corresponding to the Speed value */
		if( (GraphClear_flag == FLAG_RESET) || (GraphDraw_flag == FLAG_RESET) )
		{
			y = (uint16_t)( (-3.)/40.*BLDC1Handle.CurSpeed + 110 );		// y[pixel_y] = (-3)/40*x[rpm]+110

			x_prv = x;
			y_prv = y;

			x++;

			GraphDraw_flag = FLAG_SET;

			if(x >= 250)
			{
				GraphClear_flag = FLAG_SET;
			}
		}


//				/* Transmit Motor Speed value to PC through UART3 */
//				int16_t motorSpeed, motorSpeedAbs;
//				char sign;
//
//				motorSpeed = (int16_t)BLDC1Handle.CurSpeed;
//				motorSpeedAbs = abs(motorSpeed);
//
//				if(motorSpeed >= 0)			sign = '+';
//				else if(motorSpeed < 0)		sign = '-';
//
//				MotorSpeedStr[0] = sign;
//				MotorSpeedStr[1] = (motorSpeedAbs / 1000) + 48;
//				MotorSpeedStr[2] = ((motorSpeedAbs % 1000) / 100) + 48;
//				MotorSpeedStr[3] = ((motorSpeedAbs % 100) / 10) + 48;
//				MotorSpeedStr[4] = (motorSpeedAbs % 10) + 48;
//				MotorSpeedStr[5] = '\n';
//
//				UART_Transmit_DMA(&UART3Handle, (uint8_t*)MotorSpeedStr, strlen((char*)MotorSpeedStr));

		count1 = 0;
	}

	count1++;
}


void BLDC_PositionMode(void)
{
	static int count2 = 0;
	static int count3 = 0;


	/* Set PWM duty cycle by Position PID calculation */
	BLDC_PositionPID(&BLDC1Handle, 0.001);


	/* Calculate the Pixel corresponding to the Position value */
	if(count2 >= 10)
	{
		if( (GraphClear_flag == FLAG_RESET) || (GraphDraw_flag == FLAG_RESET) )
		{
			y = (uint16_t)( (156. * BLDC1Handle.CurPosition) / BLDC1Handle.RefPosition );

			x_prv = x;
			y_prv = y;

			x++;

			GraphDraw_flag = FLAG_SET;

			if(x >= 250)
			{
				GraphClear_flag = FLAG_SET;
			}
		}

		count2 = 0;
	}


	/* Transmit Motor Position value to PC through UART3 */
//	if(count3 >= 2)		// Every 2ms
//	{
//
//		if(BLDC1Handle.MotorState == MOTOR_STATE_POSITION)
//		{
//			// To see the case of RefPosition
//			sprintf(Msg1, "%.2lf,%.2lf\n", BLDC1Handle.RefPosition, BLDC1Handle.CurPosition);
//		}
//		else if(BLDC1Handle.MotorState == MOTOR_STATE_POSITION_TRACKING)
//		{
//			// To see the case of TrjCurPosition
//			sprintf(Msg1, "%.2lf,%.2lf\n", BLDC1Handle.TrjCurPosition, BLDC1Handle.CurPosition);
//		}
//
//
//		UART_Transmit_DMA(&UART3Handle, (uint8_t*)Msg1, strlen((char*)Msg1));
//
//		count3 = 0;
//	}


	count2++;
	count3++;
}


void Detect_KeyInput(void)
{
	/* Mode Key Press Detection */
	if(KeyFlag_Mode == FLAG_SET)
	{
		KeyTime_Mode++;

		if(KeyTime_Mode >= 20)
		{
			/* Key is still pressed */
			if(READ_BIT(GPIOA->IDR, GPIO_PIN_0) == 0)
			{
				// to be implemented
			}

			/* Key is not pressed anymore */
			else
			{
				if(Mode_key >= 2)
				{
					Mode_key = 0;
				}
				else
				{
					Mode_key++;
				}

				KeyFlag_Mode = FLAG_RESET;
				KeyTime_Mode = 0;
			}
		}
	}

	/* Up Key Press Detection */
	else if(KeyFlag_Up == FLAG_SET)
	{
		KeyTime_Up++;

		if(KeyTime_Up >= 200)
		{
			/* Key is still pressed */
			if(READ_BIT(GPIOA->IDR, GPIO_PIN_1) == 0)
			{
				// to be implemented
				Up_key++;
			}

			/* Key is not pressed anymore */
			else
			{
				Up_key++;

				KeyFlag_Up = FLAG_RESET;
				KeyTime_Up = 0;
			}
		}
	}

	/* Down Key Press Detection */
	else if(KeyFlag_Down == FLAG_SET)
	{
		KeyTime_Down++;

		if(KeyTime_Down >= 200)
		{
			/* Key is still pressed */
			if(READ_BIT(GPIOA->IDR, GPIO_PIN_2) == 0)
			{
				// to be implemented
				Down_key++;
			}

			/* Key is not pressed anymore */
			else
			{
				Down_key++;

				KeyFlag_Down = FLAG_RESET;
				KeyTime_Down = 0;
			}
		}
	}

	/* Start/Stop Key Press Detection */
	else if(KeyFlag_Start == FLAG_SET)
	{
		KeyTime_Start++;

		if(KeyTime_Start >= 100)
		{
			/* Key is still pressed */
			if(READ_BIT(GPIOA->IDR, GPIO_PIN_3) == 0)
			{
				// to be implemented
			}

			/* Key is not pressed anymore */
			else
			{
				if(Start_key >= 1)
				{
					Start_key = 0;
				}
				else
				{
					Start_key++;
				}

				KeyFlag_Start = FLAG_RESET;
				KeyTime_Start = 0;
			}
		}
	}
}


void Detect_TouchScreenInput(void)
{
	/* Touch Screen Press Detection */

	static uint16_t x_touch_prv = 0;
	static uint16_t y_touch_prv = 0;
	static uint8_t TouchLog_flag = FLAG_RESET;

	if(READ_BIT(GPIOB->IDR, GPIO_PIN_4) == 0)
	{
		TouchTime++;

		if(TouchTime >= 200)
		{
			if(TouchDetection_flag == FLAG_RESET)
			{
				if( (TS1Handle.x_touch != x_touch_prv) && (TS1Handle.y_touch != y_touch_prv) )
				{
					if( (TS1Handle.x_touch != 0) && (TS1Handle.y_touch != 0) )
					{
						xTouch_log = TS1Handle.x_touch;
						yTouch_log = TS1Handle.y_touch;

						TouchLog_flag = FLAG_SET;

						x_touch_prv = TS1Handle.x_touch;
						y_touch_prv = TS1Handle.y_touch;
					}
				}

				TouchDetection_flag = FLAG_SET;
			}
		}
	}

	else
	{
		TouchTime = 0;
		TouchDetection_flag = FLAG_RESET;
	}


	if(TouchLog_flag == FLAG_SET)
	{
		switch (State)
		{
			case STATE_MENU :
			{
				if( (xTouch_log > 40) && (xTouch_log < 280) )
				{
					// Option : Speed Mode
					if( (yTouch_log > 95) && (yTouch_log < 140) )
					{
						if(State_option == STATE_SPEED)						Start_key = 1;
						else if(State_option == STATE_POSITION)				Mode_key = 0;
						else if(State_option == STATE_POSITION_TRACKING)	Mode_key = 0;
					}

					// Option : Position Mode
					else if( (yTouch_log > 140) && (yTouch_log < 179) )
					{
						if(State_option == STATE_POSITION)					Start_key = 1;
						else if(State_option == STATE_SPEED)				Mode_key = 1;
						else if(State_option == STATE_POSITION_TRACKING)	Mode_key = 1;
					}

					// Option : Position Tracking Mode
					else if( (yTouch_log > 179) && (yTouch_log < 220) )
					{
						if(State_option == STATE_POSITION_TRACKING)			Start_key = 1;
						else if(State_option == STATE_SPEED)				Mode_key = 2;
						else if(State_option == STATE_POSITION)				Mode_key = 2;
					}
				}

				break;
			}
			case STATE_SPEED :
			{

				break;
			}
			case STATE_POSITION :
			{

				break;
			}
			case STATE_POSITION_TRACKING :
			{

				break;
			}
			default :
			{

				break;
			}
		}

		TouchLog_flag = FLAG_RESET;
	}


	/* Touch Screen Press Detection */
//		static uint8_t TouchPressSample[2] = {1, 1};
//		static uint16_t xSamples[1000] = {0,};
//		static uint16_t ySamples[1000] = {0,};
//
//		if(READ_BIT(GPIOB->IDR, GPIO_PIN_4) == 0)
//		{
//			// Touch Screen is pressed
//			TouchPressSample[1] = TouchPressSample[0];
//			TouchPressSample[0] = 0;
//		}
//		else
//		{
//			// Touch Screen is not pressed
//			TouchPressSample[1] = TouchPressSample[0];
//			TouchPressSample[0] = 1;
//		}
//
//		if( (TouchPressSample[1] == 1) && (TouchPressSample[0] == 0) )
//		{
//			// When the Touch Screen is pressed
//			TouchDetection_flag = FLAG_SET;
//			TouchTime = 0;
//		}
//		else if( (TouchPressSample[1] == 0) && (TouchPressSample[0] == 1) )
//		{
//			// When the Touch Screen is released
//			xTouch_log = xSamples[TouchTime/2];
//			yTouch_log = ySamples[TouchTime/2];
//
//			TouchDetection_flag = FLAG_RESET;
//			TouchTime = 0;
//		}
//
//		if(TouchDetection_flag == FLAG_SET)
//		{
//			xSamples[TouchTime] = TS1Handle.x_touch;
//			ySamples[TouchTime] = TS1Handle.y_touch;
//
//			TouchTime++;
//		}

}

void EXTI_Callback(uint32_t GPIO_Pin)
{
	if(GPIO_Pin & BLDC1Handle.Init.GPIO_Pins_Hall)
	{
		// 1. Detect current HallPhase location
		BLDC1Handle.HallPhase = (READ_BIT(GPIOA->IDR, BLDC1Handle.Init.GPIO_Pins_Hall)) >> 5U;

		// 2. Get current position value
		BLDC_Get_Position(&BLDC1Handle);

		// 3. Drive BLDC motor according to HallPhase location
		BLDC_Drive(&BLDC1Handle);
	}

	else if(GPIO_Pin == GPIO_PIN_0)
	{
		KeyFlag_Mode = FLAG_SET;
	}

	else if(GPIO_Pin == GPIO_PIN_1)
	{
		KeyFlag_Up = FLAG_SET;
	}

	else if(GPIO_Pin == GPIO_PIN_2)
	{
		KeyFlag_Down = FLAG_SET;
	}

	else if(GPIO_Pin == GPIO_PIN_3)
	{
		KeyFlag_Start = FLAG_SET;
	}


	UNUSED(GPIO_Pin);
}



/********************************************************************************************************************
 *							Group of functions which belong to main function for increasing Readability				*
 ********************************************************************************************************************/

void State_Menu(void)
{
	// 1. Reset all variables (structure members, global variables)
	Reset_All_Variables();

	// 2. Clear Screen of TFT LCD
	TFT_Clear_Screen(&TFT1Handle);
	Delay_ms(100);

	// 3. Draw Boundary outlines of TFT LCD
	TFT_Line(&TFT1Handle, 0, 0, 319, 0, Blue);
	TFT_Line(&TFT1Handle, 0, 1, 319, 1, Blue);
	TFT_Line(&TFT1Handle, 0, 0, 0, 239, Blue);
	TFT_Line(&TFT1Handle, 1, 0, 1, 239, Blue);
	TFT_Line(&TFT1Handle, 318, 0, 318, 239, Blue);
	TFT_Line(&TFT1Handle, 319, 0, 319, 239, Blue);
	TFT_Line(&TFT1Handle, 0, 238, 319, 238, Blue);
	TFT_Line(&TFT1Handle, 0, 239, 319, 239, Blue);

	// 4. Draw Mode option Box of TFT LCD
	TFT_Line(&TFT1Handle, 40, 95, 280, 95, Blue);
	TFT_Line(&TFT1Handle, 40, 95, 40, 220, Blue);
	TFT_Line(&TFT1Handle, 280, 95, 280, 220, Blue);
	TFT_Line(&TFT1Handle, 40, 220, 280, 220, Blue);

	// 5. Print String
	TFT_String_Large(&TFT1Handle, 6, 5, White, Magenta, (uint8_t*)"  Select mode  ");
	Delay_ms(10);


	while(1)
	{
		/* Get Touch Screen input */
		TS_Input(&TS1Handle);


		/* Select BLDC Motor Operation Mode */
		switch (Mode_key)
		{
			/* Option : Speed Mode */
			case 0 :
			{
				TFT_String(&TFT1Handle, 7, 14, White, Magenta, (uint8_t*)"        Speed Mode        ");
				TFT_String(&TFT1Handle, 7, 19, White, Black, (uint8_t*)"       Position Mode      ");
				TFT_String(&TFT1Handle, 7, 24, White, Black, (uint8_t*)"  Position Tracking Mode  ");
				State_option = STATE_SPEED;

				break;
			}

			/* Option : Position Mode */
			case 1 :
			{
				TFT_String(&TFT1Handle, 7, 14, White, Black, (uint8_t*)"        Speed Mode        ");
				TFT_String(&TFT1Handle, 7, 19, White, Magenta, (uint8_t*)"       Position Mode      ");
				TFT_String(&TFT1Handle, 7, 24, White, Black, (uint8_t*)"  Position Tracking Mode  ");
				State_option = STATE_POSITION;

				break;
			}

			/* Option : Position Tracking Mode */
			case 2 :
			{
				TFT_String(&TFT1Handle, 7, 14, White, Black, (uint8_t*)"        Speed Mode        ");
				TFT_String(&TFT1Handle, 7, 19, White, Black, (uint8_t*)"       Position Mode      ");
				TFT_String(&TFT1Handle, 7, 24, White, Magenta, (uint8_t*)"  Position Tracking Mode  ");
				State_option = STATE_POSITION_TRACKING;

				break;
			}

			default :
			{
				break;
			}
		}


		/* If 'Start' key is pressed */
		if(Start_key >= FLAG_SET)
		{
			State = State_option;
			Mode_key = FLAG_RESET;
			Up_key = 0;
			Down_key = 0;
			Start_key = FLAG_RESET;
			break;
		}


		Delay_ms(100);
	}
}


void State_Speed(void)
{
	int32_t temp_RefSpeed;

	Mode_key = FLAG_RESET;
	Start_key = FLAG_RESET;

	// 1. Clear Screen of TFT LCD
	TFT_Clear_Screen(&TFT1Handle);
	Delay_ms(100);

	// 2. Draw Axis of Graph
	Draw_axis(&TFT1Handle, State);

	// 3. Print Strings
	TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
	TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
	Delay_ms(100);


	while(1)
	{
		/* Set Reference Speed of BLDC Motor with Key inputs */
		temp_RefSpeed = Up_key - Down_key;

		if(temp_RefSpeed > 1200)			BLDC_SET_REFERENCE_SPEED(&BLDC1Handle, 1200);
		else if(temp_RefSpeed < -1200)		BLDC_SET_REFERENCE_SPEED(&BLDC1Handle, -1200);
		else								BLDC_SET_REFERENCE_SPEED(&BLDC1Handle, temp_RefSpeed);


		if(Start_key == FLAG_RESET)
		{
			if(Recharge_flag == FLAG_SET)
			{
				BLDC1Handle.MotorState = MOTOR_STATE_RECHARGE;
				BLDC_BootstrapCap_Charge(&BLDC1Handle);
				EXTI->SWIER |= (0x1 << 5);
				BLDC1Handle.MotorState = MOTOR_STATE_SPEED;

				Recharge_flag = FLAG_RESET;
			}
		}


		/* Display Speed(Reference, Current) value */
		TFT_xy(&TFT1Handle, 31, 20);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.RefSpeed, 4, 1);
		TFT_xy(&TFT1Handle, 31, 22);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.CurSpeed, 4, 1);


		/* Draw Graph of Motor Speed */
		Draw_Graph(&TFT1Handle);


		/* If 'Start' key is pressed */
		if(Start_key >= FLAG_SET)
		{
			/* Wait to Avoid Key chattering */
			Delay_ms(200);

			/* If 'Motor State' is 'STOP' */
			if(BLDC1Handle.MotorState == MOTOR_STATE_STOP)
			{
				/* 'Motor State' Changes from 'STOP' to 'SPEED' */

				// 1. Clear Graph Screen
				Clear_Graph(&TFT1Handle);
				Delay_ms(10);

				// 2. Draw Axis of Graph
				Draw_axis(&TFT1Handle, State);

				// 3. Print Strings
				TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
				TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");

				// 4. Set PID gain
				BLDC_PID_GAIN_SET(&BLDC1Handle, 0.02, 8, 0);	// 0.02, 8, 0

				// 5. Set Direction of Rotation
				if(BLDC1Handle.RefSpeed >= 0)			BLDC1Handle.RotationDir = CW;
				else if(BLDC1Handle.RefSpeed < 0)		BLDC1Handle.RotationDir = CCW;

				// 6. Set Old HallPhase location based on Current HallPhase
				BLDC_SET_OLD_HALLPHASE(&BLDC1Handle);

				// 7. Charge Bootstrap Capacitor for 10ms
				BLDC_BootstrapCap_Charge(&BLDC1Handle);

				// 8. Enable EXTI of Hall sensor
				ENABLE_HALLSENSOR_EXTI();

				// 9. Trigger EXTI interrupt by SW to Execute 'BLDC_Drive' function. (Top Logic On, Bottom PWM On. But TIM_CCR == 0)
				EXTI->SWIER |= (0x1 << 5);	// The purpose of this line is to trigger EXTI9_5_IRQHandler. So, 5 can be replaced by 6, 7.

				// 10. Reset HallCount value to 0. When EXTI9_5_IRQHandler is triggered, BLDC_Get_Position function increases / dicrease HallCount value by 1
				BLDC1Handle.HallCount = 0;

				// 11. Change 'MotorState' from 'MOTOR_STATE_STOP' to 'MOTOR_STATE_SPEED'
				BLDC1Handle.MotorState = MOTOR_STATE_SPEED;
			}


			/* If 'Motor state' is 'SPEED' */
			else if( (BLDC1Handle.MotorState == MOTOR_STATE_SPEED) || (BLDC1Handle.MotorState == MOTOR_STATE_RECHARGE) )
			{
				/* 'Motor State' Changes from 'SPEED' to 'STOP' */

				// 1. Set Reference Speed to 0
				BLDC_SET_REFERENCE_SPEED(&BLDC1Handle, 0);

				// 2. Wait for the Motor to stop completely
				while( ((int16_t)BLDC1Handle.CurSpeed) != 0 )
				{
					// Print Speed(Reference, Current) value until Motor stops completely
					TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
					TFT_xy(&TFT1Handle, 31, 20);
					TFT_Signed_float(&TFT1Handle, BLDC1Handle.RefSpeed, 4, 1);
					TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
					TFT_xy(&TFT1Handle, 31, 22);
					TFT_Signed_float(&TFT1Handle, BLDC1Handle.CurSpeed, 4, 1);
				}

				Delay_ms(100);

				// 3. Disable EXTI of Hall sensor
				DISABLE_HALLSENSOR_EXTI();

				// 4. Clear GPIO pin of Top side(UT, VT, WT)
				GPIO_WritePin(BLDC1Handle.Init.GPIOx_Top, BLDC1Handle.Init.GPIO_Pins_Top, GPIO_PIN_RESET);

				// 5. Disable All PWM channels
				DisableTimerPwmChannel(&BLDC1Handle);

				// 6. Change MotorState from MOTOR_STATE_SPEED to MOTOR_STATE_STOP
				BLDC1Handle.MotorState = MOTOR_STATE_STOP;

				// 7. Reset variables
				Reset_Speed_Variables();

				Recharge_flag = FLAG_RESET;
				Mode_key = FLAG_RESET;
			}

			Start_key = FLAG_RESET;
		}


		/* If Mode key is pressed when Motor is running, Ignore the Mode key press  */
		if( (BLDC1Handle.MotorState == MOTOR_STATE_SPEED) && (Mode_key >= FLAG_SET) )
		{
			Mode_key = FLAG_RESET;
		}

		/* If Mode key is pressed when Motor is stopped, Return to the Menu state  */
		else if( (BLDC1Handle.MotorState == MOTOR_STATE_STOP) && (Mode_key >= FLAG_SET) )
		{
			State = STATE_MENU;
			Mode_key = FLAG_RESET;
			break;
		}


	}
}


void State_Position(void)
{
	int32_t temp_RefPosition;

	Mode_key = FLAG_RESET;
	Start_key = FLAG_RESET;

	// 1. Clear Screen of TFT LCD
	TFT_Clear_Screen(&TFT1Handle);
	Delay_ms(100);

	// 2. Draw Axis of Graph
	Draw_axis(&TFT1Handle, State);

	// 3. Print Strings
	TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
	TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
	Delay_ms(100);


	while(1)
	{
		if(BLDC1Handle.MotorState == MOTOR_STATE_POSITION)
		{
			Mode_key = FLAG_RESET;
		}


		/* If 'Motor State' is 'STOP' */
		if(BLDC1Handle.MotorState == MOTOR_STATE_STOP)
		{
			// Set Reference Position of BLDC Motor with Key inputs
			temp_RefPosition = 10 * (Up_key - Down_key);

			if(temp_RefPosition > 99999)			BLDC_SET_REFERENCE_POSITION(&BLDC1Handle, 99999);
			else if(temp_RefPosition < -99999)		BLDC_SET_REFERENCE_POSITION(&BLDC1Handle, -99999);
			else									BLDC_SET_REFERENCE_POSITION(&BLDC1Handle, temp_RefPosition);

			// Display Reference Position value
			TFT_xy(&TFT1Handle, 31, 20);
			TFT_Signed_float(&TFT1Handle, BLDC1Handle.RefPosition, 5, 1);
		}


		/* Display Position(Reference, Current) value */
		TFT_xy(&TFT1Handle, 31, 20);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.RefPosition, 5, 1);
		TFT_xy(&TFT1Handle, 31, 22);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.CurPosition, 5, 1);


		/* Draw Graph of Motor Position */
		Draw_Graph(&TFT1Handle);


		/* If 'Start' key is pressed */
		if(Start_key >= FLAG_SET)
		{
			/* Wait to Avoid Key chattering */
			Delay_ms(200);

			/* If 'Motor State' is 'STOP' */
			if(BLDC1Handle.MotorState == MOTOR_STATE_STOP)
			{
				/* 'Motor State' Changes from 'STOP' to 'POSITION' */

				// 1. Clear Graph Screen
				Clear_Graph(&TFT1Handle);
				Delay_ms(10);

				// 2. Draw Axis of Graph
				Draw_axis(&TFT1Handle, State);

				// 3. Print Strings
				TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
				TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");

				// 4. Set PID gain
				BLDC_PID_GAIN_SET(&BLDC1Handle, 15, 0, 0.01);

				// 5. Set Direction of Rotation
				if(BLDC1Handle.RefPosition >= 0)			BLDC1Handle.RotationDir = CW;
				else if(BLDC1Handle.RefPosition < 0)		BLDC1Handle.RotationDir = CCW;

				// 6. Set Old HallPhase location based on Current HallPhase
				BLDC_SET_OLD_HALLPHASE(&BLDC1Handle);

				// 7. Charge Bootstrap Capacitor for 10ms
				BLDC_BootstrapCap_Charge(&BLDC1Handle);

				// 8. Enable EXTI of Hall sensor
				ENABLE_HALLSENSOR_EXTI();

				// 9. Trigger EXTI interrupt by SW to Execute 'BLDC_Drive' function. (Top Logic On, Bottom PWM On. But TIM_CCR == 0)
				EXTI->SWIER |= (0x1 << 5);	// The purpose of this line is to trigger EXTI9_5_IRQHandler. So, 5 can be replaced by 6, 7.

				// 10. Reset HallCount value to 0. When EXTI9_5_IRQHandler is triggered, BLDC_Get_Position function increases / dicrease HallCount value by 1
				BLDC1Handle.HallCount = 0;

				// 11. Change MotorState from MOTOR_STATE_STOP to MOTOR_STATE_POSITION
				BLDC1Handle.MotorState = MOTOR_STATE_POSITION;
			}


			/* If 'Motor state' is 'POSITION' */
			else if(BLDC1Handle.MotorState == MOTOR_STATE_POSITION)
			{
				/* 'Motor State' Changes from 'POSITION' to 'STOP' */

				// 1. Disable EXTI of Hall sensor
				DISABLE_HALLSENSOR_EXTI();

				// 2. Clear GPIO pin of Top side(UT, VT, WT)
				GPIO_WritePin(BLDC1Handle.Init.GPIOx_Top, BLDC1Handle.Init.GPIO_Pins_Top, GPIO_PIN_RESET);

				// 3. Disable All PWM channels
				DisableTimerPwmChannel(&BLDC1Handle);

				// 4. Change MotorState from MOTOR_STATE_POSITION to MOTOR_STATE_STOP
				BLDC1Handle.MotorState = MOTOR_STATE_STOP;

				// 5. Reset variables
				Reset_Position_Variables();

				Mode_key = 0;
				Up_key = 0;
				Down_key = 0;
			}

			Start_key = FLAG_RESET;
		}


		/* If Mode key is pressed when Motor is running, Ignore the Mode key press  */
		if( (BLDC1Handle.MotorState == MOTOR_STATE_POSITION) && (Mode_key >= FLAG_SET) )
		{
			Mode_key = FLAG_RESET;
		}

		/* If Mode key is pressed when Motor is stopped, Return to the Menu state  */
		else if( (BLDC1Handle.MotorState == MOTOR_STATE_STOP) && (Mode_key >= FLAG_SET) )
		{
			State = STATE_MENU;
			Mode_key = FLAG_RESET;
			break;
		}
	}
}


void State_Position_Tracking(void)
{
	int32_t temp_RefPosition;

	Mode_key = FLAG_RESET;
	Start_key = FLAG_RESET;

	// 1. Clear Screen of TFT LCD
	TFT_Clear_Screen(&TFT1Handle);
	Delay_ms(100);

	// 2. Draw Axis of Graph
	Draw_axis(&TFT1Handle, State);

	// 3. Print Strings
	TFT_String(&TFT1Handle, 25, 18, White, Black, (uint8_t*)"Ref : ");
	TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Trj : ");
	TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
	Delay_ms(100);


	while(1)
	{
		if(BLDC1Handle.MotorState == MOTOR_STATE_POSITION_TRACKING)
		{
			Mode_key = FLAG_RESET;
		}


		/* If 'Motor State' is 'STOP' */
		if(BLDC1Handle.MotorState == MOTOR_STATE_STOP)
		{
			// Set Reference Position of BLDC Motor with Key inputs
			temp_RefPosition = 10 * (Up_key - Down_key);

			if(temp_RefPosition > 99999)			BLDC_SET_REFERENCE_POSITION(&BLDC1Handle, 99999);
			else if(temp_RefPosition < -99999)		BLDC_SET_REFERENCE_POSITION(&BLDC1Handle, -99999);
			else									BLDC_SET_REFERENCE_POSITION(&BLDC1Handle, temp_RefPosition);

			BLDC1Handle.TrjRefMaxSpeed = 7500;
			BLDC1Handle.TrjRefAcceleration = 2000;

			// Display Reference Position value
			TFT_xy(&TFT1Handle, 31, 18);
			TFT_Signed_float(&TFT1Handle, BLDC1Handle.RefPosition, 5, 1);
		}


		/* Display (Reference, Trajectory Current, Current) Position value */
		TFT_xy(&TFT1Handle, 31, 18);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.RefPosition, 5, 1);
		TFT_xy(&TFT1Handle, 31, 20);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.TrjCurPosition, 5, 1);
		TFT_xy(&TFT1Handle, 31, 22);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.CurPosition, 5, 1);


		/* Draw Graph of Motor Position */
		Draw_Graph(&TFT1Handle);


		/* If 'Start' key is pressed */
		if(Start_key >= FLAG_SET)
		{
			/* Wait to Avoid Key chattering */
			Delay_ms(200);

			/* If 'Motor State' is 'STOP' */
			if(BLDC1Handle.MotorState == MOTOR_STATE_STOP)
			{
				/* 'Motor State' Changes from 'STOP' to 'POSITION TRACKING' */

				// 1. Clear Graph Screen
				Clear_Graph(&TFT1Handle);
				Delay_ms(10);

				// 2. Draw Axis of Graph
				Draw_axis(&TFT1Handle, State);

				// 3. Print Strings
				TFT_String(&TFT1Handle, 25, 18, White, Black, (uint8_t*)"Ref : ");
				TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Trj : ");
				TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");

				// 4. Set PID gain
				BLDC_PID_GAIN_SET(&BLDC1Handle, 25, 4, 0);		// 25, 4, 0

				// 5. Set Direction of Rotation
				if(BLDC1Handle.RefPosition >= 0)			BLDC1Handle.RotationDir = CW;
				else if(BLDC1Handle.RefPosition < 0)		BLDC1Handle.RotationDir = CCW;

				// 6. Set Old HallPhase location based on Current HallPhase
				BLDC_SET_OLD_HALLPHASE(&BLDC1Handle);

				// 7. Charge Bootstrap Capacitor for 10ms
				BLDC_BootstrapCap_Charge(&BLDC1Handle);

				// 8. Enable EXTI of Hall sensor
				ENABLE_HALLSENSOR_EXTI();

				// 9. Trigger EXTI interrupt by SW to Execute 'BLDC_Drive' function. (Top Logic On, Bottom PWM On. But TIM_CCR == 0)
				EXTI->SWIER |= (0x1 << 5);		// The purpose of this line is to trigger EXTI9_5_IRQHandler. So, 5 can be replaced by 6, 7.

				// 10. Reset HallCount value to 0. When EXTI9_5_IRQHandler is triggered, BLDC_Get_Position function increases / dicrease HallCount value by 1
				BLDC1Handle.HallCount = 0;

				// 11. Change MotorState from MOTOR_STATE_STOP to MOTOR_STATE_POSITION_TRACKING
				BLDC1Handle.MotorState = MOTOR_STATE_POSITION_TRACKING;
			}


			/* If 'Motor state' is 'POSITION_TRACKING' */
			else if(BLDC1Handle.MotorState == MOTOR_STATE_POSITION_TRACKING)
			{
				// 1. Disable EXTI of Hall sensor
				DISABLE_HALLSENSOR_EXTI();

				// 2. Clear GPIO pin of Top side(UT, VT, WT)
				GPIO_WritePin(BLDC1Handle.Init.GPIOx_Top, BLDC1Handle.Init.GPIO_Pins_Top, GPIO_PIN_RESET);

				// 3. Disable All PWM channels
				DisableTimerPwmChannel(&BLDC1Handle);

				// 4. Change MotorState from MOTOR_STATE_POSITION_TRACKING to MOTOR_STATE_STOP
				BLDC1Handle.MotorState = MOTOR_STATE_STOP;

				// 5. Reset variables
				Reset_Position_Variables();

				Mode_key = 0;
				Up_key = 0;
				Down_key = 0;
			}

			Start_key = FLAG_RESET;
		}


		/* If Mode key is pressed when Motor is running, Ignore the Mode key press  */
		if( (BLDC1Handle.MotorState == MOTOR_STATE_POSITION_TRACKING) && (Mode_key >= FLAG_SET) )
		{
			Mode_key = FLAG_RESET;
		}

		/* If Mode key is pressed when Motor is stopped, Return to the Menu state  */
		else if( (BLDC1Handle.MotorState == MOTOR_STATE_STOP) && (Mode_key >= FLAG_SET) )
		{
			State = STATE_MENU;
			Mode_key = FLAG_RESET;
			break;
		}
	}
}



void Reset_All_Variables(void)
{
	// Reset Global Structure members
	BLDC1Handle.MotorState = MOTOR_STATE_STOP;
	BLDC1Handle.HallCount = 0;
	BLDC1Handle.OldHallCount = 0;
	BLDC1Handle.CurSpeed = 0;
	BLDC1Handle.RefSpeed = 0;
	BLDC1Handle.CurPosition = 0;
	BLDC1Handle.RefPosition = 0;
	BLDC1Handle.PrvRefPosition = 0;
	BLDC1Handle.TrjCurPosition = 0;
	BLDC1Handle.TrjCurSpeed = 0;
	BLDC1Handle.TrjRefMaxSpeed = 0;
	BLDC1Handle.TrjRefAcceleration = 0;
	BLDC1Handle.TrjDtAcceleration = 0;
	BLDC1Handle.Kp = 0;
	BLDC1Handle.Ki = 0;
	BLDC1Handle.Kd = 0;
	BLDC1Handle.Error = 0;
	BLDC1Handle.PrvError = 0;
	BLDC1Handle.P_term = 0;
	BLDC1Handle.I_term = 0;
	BLDC1Handle.D_term = 0;
	BLDC1Handle.PwmPID = 0;

	// Reset Graph variables
	TFT_Color(&TFT1Handle, White, Black);
	x = 0;
	y = 0;
	x_prv = 0;
	y_prv = 0;

	// Reset Key variables
	Mode_key = 0;
	Up_key = 0;
	Down_key = 0;
	Start_key = 0;
}


void Reset_Speed_Variables(void)
{
	BLDC1Handle.HallCount = 0;
	BLDC1Handle.OldHallCount = 0;
	BLDC1Handle.CurSpeed = 0;
	BLDC1Handle.RefSpeed = 0;
	BLDC1Handle.Kp = 0;
	BLDC1Handle.Ki = 0;
	BLDC1Handle.Kd = 0;
	BLDC1Handle.Error = 0;
	BLDC1Handle.PrvError = 0;
	BLDC1Handle.P_term = 0;
	BLDC1Handle.I_term = 0;
	BLDC1Handle.D_term = 0;
	BLDC1Handle.PwmPID = 0;

	/* Graph variables */
	TFT_Color(&TFT1Handle, White, Black);
	x = 0;
	y = 0;
	x_prv = 0;
	y_prv = 0;
}


void Reset_Position_Variables(void)
{
	BLDC1Handle.HallCount = 0;
	BLDC1Handle.OldHallCount = 0;
	BLDC1Handle.CurPosition = 0;
	BLDC1Handle.RefPosition = 0;
	BLDC1Handle.PrvRefPosition = 0;
	BLDC1Handle.TrjCurPosition = 0;
	BLDC1Handle.TrjCurSpeed = 0;
	BLDC1Handle.TrjRefMaxSpeed = 0;
	BLDC1Handle.TrjRefAcceleration = 0;
	BLDC1Handle.TrjDtAcceleration = 0;
	BLDC1Handle.Kp = 0;
	BLDC1Handle.Ki = 0;
	BLDC1Handle.Kd = 0;
	BLDC1Handle.Error = 0;
	BLDC1Handle.PrvError = 0;
	BLDC1Handle.P_term = 0;
	BLDC1Handle.I_term = 0;
	BLDC1Handle.D_term = 0;
	BLDC1Handle.PwmPID = 0;

	/* Graph variables */
	TFT_Color(&TFT1Handle, White, Black);
	x = 0;
	y = 0;
	x_prv = 0;
	y_prv = 0;
}


void Draw_axis(TFT_HandleTypeDef *pTFTHandle, uint8_t state)
{
	if(state == STATE_SPEED)
	{
		TFT_String(pTFTHandle, 16, 0, White, Blue, (uint8_t *)"Speed Graph");


		/* X-axis */

		// 1. Draw x-axis
		TFT_Line(pTFTHandle, 50, 216, 310, 216, White);		// straight line of x-axis
		TFT_Line(pTFTHandle, 305, 211, 310, 216, White);	// upper arrow head of x-axis
		TFT_Line(pTFTHandle, 305, 221, 310, 216, White);	// lower arrow head of x-axis
		TFT_Line(pTFTHandle, 100, 216, 100, 220, White); 	// gradation for 5[s]
		TFT_Line(pTFTHandle, 150, 216, 150, 220, White); 	// gradation for 10[s]
		TFT_Line(pTFTHandle, 200, 216, 200, 220, White); 	// gradation for 15[s]
		TFT_Line(pTFTHandle, 250, 216, 250, 220, White); 	// gradation for 20[s]
		TFT_Line(pTFTHandle, 300, 216, 300, 220, White); 	// gradation for 25[s]


		// 2. Draw dotted lines for x-axis
		for(int i = 100; i <= 300; i = i + 50)
		{
			for(int j = 20; j <= 215; j = j + 5)
			{
				TFT_Pixel(pTFTHandle, i, j, White);
			}
		}


		// 3. Print Time label[s] corresponding to the x-axis gradation
		TFT_Color(pTFTHandle, Cyan, Black);
		TFT_English_pixel(pTFTHandle, 35, 222, '0');		// time label of 0[s]
		TFT_English_pixel(pTFTHandle, 97, 222, '5');		// time label of 5[s]
		TFT_English_pixel(pTFTHandle, 143, 222, '1');		// time label of 10[s]
		TFT_English_pixel(pTFTHandle, 151, 222, '0');
		TFT_English_pixel(pTFTHandle, 193, 222, '1');		// time label of 15[s]
		TFT_English_pixel(pTFTHandle, 201, 222, '5');
		TFT_English_pixel(pTFTHandle, 243, 222, '2');		// time label of 20[s]
		TFT_English_pixel(pTFTHandle, 251, 222, '0');
		TFT_English_pixel(pTFTHandle, 293, 222, '2');		// time label of 25[s]
		TFT_English_pixel(pTFTHandle, 301, 222, '5');


		// 4. Print Time unit[s] to x-axis
		TFT_Color(pTFTHandle, Magenta, Black);
		TFT_English_pixel(pTFTHandle, 288, 222, '[');		// time unit[s]
		TFT_English_pixel(pTFTHandle, 296, 222, 's');
		TFT_English_pixel(pTFTHandle, 304, 222, ']');



		/* Y-axis */

		// 1. Draw y-axis
		TFT_Line(pTFTHandle, 49, 215, 49, 5, White);		// straight line of y-axis
		TFT_Line(pTFTHandle, 44, 10, 49, 5, White);			// left arrow head of y-axis
		TFT_Line(pTFTHandle, 54, 10, 49, 5, White);			// right arrow head of y-axis
		TFT_Line(pTFTHandle, 45, 20, 49, 20, White);		// gradation for +1200[rpm]
		TFT_Line(pTFTHandle, 45, 50, 49, 50, White);		// gradation for +800[rpm]
		TFT_Line(pTFTHandle, 45, 80, 49, 80, White);		// gradation for +400[rpm]
		TFT_Line(pTFTHandle, 45, 110, 49, 110, White);		// gradation for 0[rpm]
		TFT_Line(pTFTHandle, 45, 140, 49, 140, White);		// gradation for -400[rpm]
		TFT_Line(pTFTHandle, 45, 170, 49, 170, White);		// gradation for -800[rpm]
		TFT_Line(pTFTHandle, 45, 200, 49, 200, White);		// gradation for -1200[rpm]


		// 2. Draw dotted lines for y-axis
		for(int j = 20; j <= 200; j = j + 30)
		{
			for(int i = 50; i <= 310; i = i + 5)
			{
				TFT_Pixel(pTFTHandle, i, j, White);
			}
		}

		// 3. Print Speed label[rpm] corresponding to the y-axis gradation
		TFT_Color(&TFT1Handle, Cyan, Black);
		TFT_English_pixel(pTFTHandle, 0, 14, '+');			// Speed label of +1200[rpm]
		TFT_English_pixel(pTFTHandle, 8, 14, '1');
		TFT_English_pixel(pTFTHandle, 16, 14, '2');
		TFT_English_pixel(pTFTHandle, 24, 14, '0');
		TFT_English_pixel(pTFTHandle, 32, 14, '0');
		TFT_English_pixel(pTFTHandle, 8, 44, '+');			// Speed label of +800[rpm]
		TFT_English_pixel(pTFTHandle, 16, 44, '8');
		TFT_English_pixel(pTFTHandle, 24, 44, '0');
		TFT_English_pixel(pTFTHandle, 32, 44, '0');
		TFT_English_pixel(pTFTHandle, 8, 74, '+');			// Speed label of +400[rpm]
		TFT_English_pixel(pTFTHandle, 16, 74, '4');
		TFT_English_pixel(pTFTHandle, 24, 74, '0');
		TFT_English_pixel(pTFTHandle, 32, 74, '0');
		TFT_English_pixel(pTFTHandle, 32, 104, '0');		// Speed label of 0[rpm]
		TFT_English_pixel(pTFTHandle, 8, 134, '-');			// Speed label of -400[rpm]
		TFT_English_pixel(pTFTHandle, 16, 134, '4');
		TFT_English_pixel(pTFTHandle, 24, 134, '0');
		TFT_English_pixel(pTFTHandle, 32, 134, '0');
		TFT_English_pixel(pTFTHandle, 8, 164, '-');			// Speed label of -800[rpm]
		TFT_English_pixel(pTFTHandle, 16, 164, '8');
		TFT_English_pixel(pTFTHandle, 24, 164, '0');
		TFT_English_pixel(pTFTHandle, 32, 164, '0');
		TFT_English_pixel(pTFTHandle, 0, 194, '-');			// Speed label of -1200[rpm]
		TFT_English_pixel(pTFTHandle, 8, 194, '1');
		TFT_English_pixel(pTFTHandle, 16, 194, '2');
		TFT_English_pixel(pTFTHandle, 24, 194, '0');
		TFT_English_pixel(pTFTHandle, 32, 194, '0');


		// 4. Print Speed unit[rpm] to y-axis
		TFT_Color(pTFTHandle, Magenta, Black);
		TFT_English_pixel(pTFTHandle, 0, 0, '[');
		TFT_English_pixel(pTFTHandle, 8, 0, 'r');
		TFT_English_pixel(pTFTHandle, 16, 0, 'p');
		TFT_English_pixel(pTFTHandle, 24, 0, 'm');
		TFT_English_pixel(pTFTHandle, 32, 0, ']');
	}

	else if(state == STATE_POSITION)
	{
		TFT_String(pTFTHandle, 14, 0, White, Blue, (uint8_t *)"Position Graph");


		/* X-axis */

		// 1. Draw x-axis
		TFT_Line(pTFTHandle, 50, 216, 310, 216, White);		// straight line of x-axis
		TFT_Line(pTFTHandle, 305, 211, 310, 216, White);	// upper arrow head of x-axis
		TFT_Line(pTFTHandle, 305, 221, 310, 216, White);	// lower arrow head of x-axis
		TFT_Line(pTFTHandle, 100, 216, 100, 220, White);	// gradation for 0.6[s]
		TFT_Line(pTFTHandle, 150, 216, 150, 220, White);	// gradation for 1.2[s]
		TFT_Line(pTFTHandle, 200, 216, 200, 220, White);	// gradation for 1.8[s]
		TFT_Line(pTFTHandle, 250, 216, 250, 220, White);	// gradation for 2.4[s]
		TFT_Line(pTFTHandle, 300, 216, 300, 220, White);	// gradation for 3.0[s]


		// 2. Draw dotted lines for x-axis
		for(int i = 100; i <= 300; i = i + 50)
		{
			for(int j = 20; j <= 215; j = j + 5)
			{
				TFT_Pixel(pTFTHandle, i, j, White);
			}
		}


		// 3. Print Time label[s] corresponding to the x-axis gradation
		TFT_Color(pTFTHandle, Cyan, Black);
		TFT_English_pixel(pTFTHandle, 35, 222, '0');		// time label of 0[s]
		TFT_English_pixel(pTFTHandle, 89, 222, '0');		// time label of 0.6[s]
		TFT_English_pixel(pTFTHandle, 97, 222, '.');
		TFT_English_pixel(pTFTHandle, 105, 222, '6');
		TFT_English_pixel(pTFTHandle, 139, 222, '1');		// time label of 1.2[s]
		TFT_English_pixel(pTFTHandle, 147, 222, '.');
		TFT_English_pixel(pTFTHandle, 155, 222, '2');
		TFT_English_pixel(pTFTHandle, 189, 222, '1');		// time label of 1.8[s]
		TFT_English_pixel(pTFTHandle, 197, 222, '.');
		TFT_English_pixel(pTFTHandle, 206, 222, '8');
		TFT_English_pixel(pTFTHandle, 239, 222, '2');		// time label of 2.4[s]
		TFT_English_pixel(pTFTHandle, 247, 222, '.');
		TFT_English_pixel(pTFTHandle, 255, 222, '4');


		// 4. Print Time unit[s] to x-axis
		TFT_Color(pTFTHandle, Magenta, Black);
		TFT_English_pixel(pTFTHandle, 288, 222, '[');		// time unit[s]
		TFT_English_pixel(pTFTHandle, 296, 222, 's');
		TFT_English_pixel(pTFTHandle, 304, 222, ']');



		/* Y-axis */

		// 1. Draw y-axis
		TFT_Line(pTFTHandle, 49, 215, 49, 5, White);		// straight line of y-axis
		TFT_Line(pTFTHandle, 44, 10, 49, 5, White);			// left arrow head of y-axis
		TFT_Line(pTFTHandle, 54, 10, 49, 5, White);			// right arrow head of y-axis
		TFT_Line(pTFTHandle, 45, 20, 49, 20, White);		// gradation for RefPosition * 5 / 4 [deg]
		TFT_Line(pTFTHandle, 45, 59, 49, 59, White);		// gradation for RefPosition * 4 / 4 [deg]
		TFT_Line(pTFTHandle, 45, 98, 49, 98, White);		// gradation for RefPosition * 3 / 4 [deg]
		TFT_Line(pTFTHandle, 45, 137, 49, 137, White);		// gradation for RefPosition * 2 / 4 [deg]
		TFT_Line(pTFTHandle, 45, 176, 49, 176, White);		// gradation for RefPosition * 1 / 4 [deg]

		// 2. Draw dotted lines for y-axis
		for(int j = 20; j <= 176; j = j + 39)
		{
			for(int i = 50; i <= 310; i = i + 5)
			{
				TFT_Pixel(pTFTHandle, i, j, White);
			}
		}


		// 3. Print Position label[deg] corresponding to the y-axis gradation
		// -> it will be implemented in main function because the position label values depend on user key inputs


		// 4. Print Position unit[deg] to y-axis
		TFT_Color(pTFTHandle, Magenta, Black);
		TFT_English_pixel(pTFTHandle, 0, 0, '[');
		TFT_English_pixel(pTFTHandle, 8, 0, 'd');
		TFT_English_pixel(pTFTHandle, 16, 0, 'e');
		TFT_English_pixel(pTFTHandle, 24, 0, 'g');
		TFT_English_pixel(pTFTHandle, 32, 0, ']');
	}

	else if(state == STATE_POSITION_TRACKING)
	{
		TFT_String(pTFTHandle, 10, 0, White, Blue, (uint8_t *)"Position Tracking Graph");


		/* X-axis */

		// 1. Draw x-axis
		TFT_Line(pTFTHandle, 50, 216, 310, 216, White);		// straight line of x-axis
		TFT_Line(pTFTHandle, 305, 211, 310, 216, White);	// upper arrow head of x-axis
		TFT_Line(pTFTHandle, 305, 221, 310, 216, White);	// lower arrow head of x-axis
		TFT_Line(pTFTHandle, 100, 216, 100, 220, White);	// gradation for 0.6[s]
		TFT_Line(pTFTHandle, 150, 216, 150, 220, White);	// gradation for 1.2[s]
		TFT_Line(pTFTHandle, 200, 216, 200, 220, White);	// gradation for 1.8[s]
		TFT_Line(pTFTHandle, 250, 216, 250, 220, White);	// gradation for 2.4[s]
		TFT_Line(pTFTHandle, 300, 216, 300, 220, White);	// gradation for 3.0[s]


		// 2. Draw dotted lines for x-axis
		for(int i = 100; i <= 300; i = i + 50)
		{
			for(int j = 20; j <= 215; j = j + 5)
			{
				TFT_Pixel(pTFTHandle, i, j, White);
			}
		}


		// 3. Print Time label[s] corresponding to the x-axis gradation
		TFT_Color(pTFTHandle, Cyan, Black);
		TFT_English_pixel(pTFTHandle, 35, 222, '0');		// time label of 0[s]
		TFT_English_pixel(pTFTHandle, 89, 222, '0');		// time label of 0.6[s]
		TFT_English_pixel(pTFTHandle, 97, 222, '.');
		TFT_English_pixel(pTFTHandle, 105, 222, '6');
		TFT_English_pixel(pTFTHandle, 139, 222, '1');		// time label of 1.2[s]
		TFT_English_pixel(pTFTHandle, 147, 222, '.');
		TFT_English_pixel(pTFTHandle, 155, 222, '2');
		TFT_English_pixel(pTFTHandle, 189, 222, '1');		// time label of 1.8[s]
		TFT_English_pixel(pTFTHandle, 197, 222, '.');
		TFT_English_pixel(pTFTHandle, 206, 222, '8');
		TFT_English_pixel(pTFTHandle, 239, 222, '2');		// time label of 2.4[s]
		TFT_English_pixel(pTFTHandle, 247, 222, '.');
		TFT_English_pixel(pTFTHandle, 255, 222, '4');


		// 4. Print Time unit[s] to x-axis
		TFT_Color(pTFTHandle, Magenta, Black);
		TFT_English_pixel(pTFTHandle, 288, 222, '[');		// time unit[s]
		TFT_English_pixel(pTFTHandle, 296, 222, 's');
		TFT_English_pixel(pTFTHandle, 304, 222, ']');



		/* Y-axis */

		// 1. Draw y-axis
		TFT_Line(pTFTHandle, 49, 215, 49, 5, White);		// straight line of y-axis
		TFT_Line(pTFTHandle, 44, 10, 49, 5, White);			// left arrow head of y-axis
		TFT_Line(pTFTHandle, 54, 10, 49, 5, White);			// right arrow head of y-axis
		TFT_Line(pTFTHandle, 45, 20, 49, 20, White);		// gradation for RefPosition * 5 / 4 [deg]
		TFT_Line(pTFTHandle, 45, 59, 49, 59, White);		// gradation for RefPosition * 4 / 4 [deg]
		TFT_Line(pTFTHandle, 45, 98, 49, 98, White);		// gradation for RefPosition * 3 / 4 [deg]
		TFT_Line(pTFTHandle, 45, 137, 49, 137, White);		// gradation for RefPosition * 2 / 4 [deg]
		TFT_Line(pTFTHandle, 45, 176, 49, 176, White);		// gradation for RefPosition * 1 / 4 [deg]


		// 2. Draw dotted lines for y-axis
		for(int j = 20; j <= 176; j = j + 39)
		{
			for(int i = 50; i <= 310; i = i + 5)
			{
				TFT_Pixel(pTFTHandle, i, j, White);
			}
		}


		// 3. Print Position label[deg] corresponding to the y-axis gradation
		// -> it will be implemented in main function because the position label values depend on user key inputs


		// 4. Print Position unit[deg] to y-axis
		TFT_Color(pTFTHandle, Magenta, Black);
		TFT_English_pixel(pTFTHandle, 0, 0, '[');
		TFT_English_pixel(pTFTHandle, 8, 0, 'd');
		TFT_English_pixel(pTFTHandle, 16, 0, 'e');
		TFT_English_pixel(pTFTHandle, 24, 0, 'g');
		TFT_English_pixel(pTFTHandle, 32, 0, ']');
	}
}


void Draw_Graph(TFT_HandleTypeDef *pTFTHandle)
{
	switch (State)
	{
		case STATE_SPEED :
		{
			if(GraphClear_flag == FLAG_SET)
			{
				// Erase Graph of Current Position
				Clear_Graph(&TFT1Handle);

				x = 0;
				y = 0;
				x_prv = 0;
				y_prv = 0;

				GraphClear_flag = FLAG_RESET;

				TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
				TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
			}
			else if(GraphDraw_flag == FLAG_SET)
			{
				// Draw Graph of Current Position
				TFT_Line(&TFT1Handle, 50+x_prv, y_prv, 50+x, y, Green);
				GraphDraw_flag = FLAG_RESET;
			}


			break;
		}

		case STATE_POSITION :
		{
			if(GraphClear_flag == FLAG_SET)
			{
				// Erase Graph of Current Position
				Clear_Graph(&TFT1Handle);

				x = 0;
				y = 0;
				x_prv = 0;
				y_prv = 0;

				GraphClear_flag = FLAG_RESET;

				TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
				TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
			}
			else if(GraphDraw_flag == FLAG_SET)
			{
				// Draw Graph of Current Position
				TFT_Line(&TFT1Handle, 50+x_prv, 215-y_prv, 50+x, 215-y, Green);	// (50,215) is the origin of Graph
				GraphDraw_flag = FLAG_RESET;
			}


			if(BLDC1Handle.MotorState == MOTOR_STATE_STOP)
			{
				// Label axis value
				char temp_str[7] = {0,};

				TFT_Color(&TFT1Handle, Cyan, Black);

				sprintf(temp_str, "%+6ld", (int32_t)(BLDC1Handle.RefPosition * 5. / 4.));
				TFT_English_pixel(&TFT1Handle, 0, 14, temp_str[0]);
				TFT_English_pixel(&TFT1Handle, 8, 14, temp_str[1]);
				TFT_English_pixel(&TFT1Handle, 16, 14, temp_str[2]);
				TFT_English_pixel(&TFT1Handle, 24, 14, temp_str[3]);
				TFT_English_pixel(&TFT1Handle, 32, 14, temp_str[4]);
				TFT_English_pixel(&TFT1Handle, 40, 14, temp_str[5]);

				sprintf(temp_str, "%+6ld", (int32_t)(BLDC1Handle.RefPosition * 4. / 4.));
				TFT_English_pixel(&TFT1Handle, 0, 53, temp_str[0]);
				TFT_English_pixel(&TFT1Handle, 8, 53, temp_str[1]);
				TFT_English_pixel(&TFT1Handle, 16, 53, temp_str[2]);
				TFT_English_pixel(&TFT1Handle, 24, 53, temp_str[3]);
				TFT_English_pixel(&TFT1Handle, 32, 53, temp_str[4]);
				TFT_English_pixel(&TFT1Handle, 40, 53, temp_str[5]);

				sprintf(temp_str, "%+6ld", (int32_t)(BLDC1Handle.RefPosition * 3. / 4.));
				TFT_English_pixel(&TFT1Handle, 0, 92, temp_str[0]);
				TFT_English_pixel(&TFT1Handle, 8, 92, temp_str[1]);
				TFT_English_pixel(&TFT1Handle, 16, 92, temp_str[2]);
				TFT_English_pixel(&TFT1Handle, 24, 92, temp_str[3]);
				TFT_English_pixel(&TFT1Handle, 32, 92, temp_str[4]);
				TFT_English_pixel(&TFT1Handle, 40, 92, temp_str[5]);

				sprintf(temp_str, "%+6ld", (int32_t)(BLDC1Handle.RefPosition * 2. / 4.));
				TFT_English_pixel(&TFT1Handle, 0, 131, temp_str[0]);
				TFT_English_pixel(&TFT1Handle, 8, 131, temp_str[1]);
				TFT_English_pixel(&TFT1Handle, 16, 131, temp_str[2]);
				TFT_English_pixel(&TFT1Handle, 24, 131, temp_str[3]);
				TFT_English_pixel(&TFT1Handle, 32, 131, temp_str[4]);
				TFT_English_pixel(&TFT1Handle, 40, 131, temp_str[5]);

				sprintf(temp_str, "%+6ld", (int32_t)(BLDC1Handle.RefPosition * 1. / 4.));
				TFT_English_pixel(&TFT1Handle, 0, 170, temp_str[0]);
				TFT_English_pixel(&TFT1Handle, 8, 170, temp_str[1]);
				TFT_English_pixel(&TFT1Handle, 16, 170, temp_str[2]);
				TFT_English_pixel(&TFT1Handle, 24, 170, temp_str[3]);
				TFT_English_pixel(&TFT1Handle, 32, 170, temp_str[4]);
				TFT_English_pixel(&TFT1Handle, 40, 170, temp_str[5]);
			}


			break;
		}

		case STATE_POSITION_TRACKING :
		{
			if(GraphClear_flag == FLAG_SET)
			{
				// Erase Graph of Current Position
				Clear_Graph(&TFT1Handle);

				x = 0;
				y = 0;
				x_prv = 0;
				y_prv = 0;

				GraphClear_flag = FLAG_RESET;

				TFT_String(&TFT1Handle, 25, 18, White, Black, (uint8_t*)"Ref : ");
				TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Trj : ");
				TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
			}
			else if(GraphDraw_flag == FLAG_SET)
			{
				// Draw Graph of Current Position
				TFT_Line(&TFT1Handle, 50+x_prv, 215-y_prv, 50+x, 215-y, Green);
				GraphDraw_flag = FLAG_RESET;
			}


			if(BLDC1Handle.MotorState == MOTOR_STATE_STOP)
			{
				// Label axis value
				char temp_str[7] = {0,};

				TFT_Color(&TFT1Handle, Cyan, Black);

				sprintf(temp_str, "%+6ld", (int32_t)(BLDC1Handle.RefPosition * 5. / 4.));
				TFT_English_pixel(&TFT1Handle, 0, 14, temp_str[0]);
				TFT_English_pixel(&TFT1Handle, 8, 14, temp_str[1]);
				TFT_English_pixel(&TFT1Handle, 16, 14, temp_str[2]);
				TFT_English_pixel(&TFT1Handle, 24, 14, temp_str[3]);
				TFT_English_pixel(&TFT1Handle, 32, 14, temp_str[4]);
				TFT_English_pixel(&TFT1Handle, 40, 14, temp_str[5]);

				sprintf(temp_str, "%+6ld", (int32_t)(BLDC1Handle.RefPosition * 4. / 4.));
				TFT_English_pixel(&TFT1Handle, 0, 53, temp_str[0]);
				TFT_English_pixel(&TFT1Handle, 8, 53, temp_str[1]);
				TFT_English_pixel(&TFT1Handle, 16, 53, temp_str[2]);
				TFT_English_pixel(&TFT1Handle, 24, 53, temp_str[3]);
				TFT_English_pixel(&TFT1Handle, 32, 53, temp_str[4]);
				TFT_English_pixel(&TFT1Handle, 40, 53, temp_str[5]);

				sprintf(temp_str, "%+6ld", (int32_t)(BLDC1Handle.RefPosition * 3. / 4.));
				TFT_English_pixel(&TFT1Handle, 0, 92, temp_str[0]);
				TFT_English_pixel(&TFT1Handle, 8, 92, temp_str[1]);
				TFT_English_pixel(&TFT1Handle, 16, 92, temp_str[2]);
				TFT_English_pixel(&TFT1Handle, 24, 92, temp_str[3]);
				TFT_English_pixel(&TFT1Handle, 32, 92, temp_str[4]);
				TFT_English_pixel(&TFT1Handle, 40, 92, temp_str[5]);

				sprintf(temp_str, "%+6ld", (int32_t)(BLDC1Handle.RefPosition * 2. / 4.));
				TFT_English_pixel(&TFT1Handle, 0, 131, temp_str[0]);
				TFT_English_pixel(&TFT1Handle, 8, 131, temp_str[1]);
				TFT_English_pixel(&TFT1Handle, 16, 131, temp_str[2]);
				TFT_English_pixel(&TFT1Handle, 24, 131, temp_str[3]);
				TFT_English_pixel(&TFT1Handle, 32, 131, temp_str[4]);
				TFT_English_pixel(&TFT1Handle, 40, 131, temp_str[5]);

				sprintf(temp_str, "%+6ld", (int32_t)(BLDC1Handle.RefPosition * 1. / 4.));
				TFT_English_pixel(&TFT1Handle, 0, 170, temp_str[0]);
				TFT_English_pixel(&TFT1Handle, 8, 170, temp_str[1]);
				TFT_English_pixel(&TFT1Handle, 16, 170, temp_str[2]);
				TFT_English_pixel(&TFT1Handle, 24, 170, temp_str[3]);
				TFT_English_pixel(&TFT1Handle, 32, 170, temp_str[4]);
				TFT_English_pixel(&TFT1Handle, 40, 170, temp_str[5]);
			}


			break;
		}

		default :
		{
			break;
		}
	}
}


void Clear_Graph(TFT_HandleTypeDef *pTFTHandle)
{
	/* Window Re-Setting */

	// 1. x = 50 ~ 300
	TFT_Write(pTFTHandle, 0x02, 50U >> 8);
	TFT_Write(pTFTHandle, 0x03, 50U & 0x00FF);
	TFT_Write(pTFTHandle, 0x04, 300U >> 8);
	TFT_Write(pTFTHandle, 0x05, 300U & 0x00FF);


	// 2. y = 15 ~ 215
	TFT_Write(pTFTHandle, 0x06, 0x0000);
	TFT_Write(pTFTHandle, 0x07, 15U);
	TFT_Write(pTFTHandle, 0x08, 0x0000);
	TFT_Write(pTFTHandle, 0x09, 215U);

	TFT_Command(pTFTHandle, 0x22);



	/* Clear graph */

	// 1. Fill the Window with Black
	for(uint16_t i = 0; i < 251; i++)
	{
		for(uint16_t j = 0; j < 201; j++)
		{
			TFT_Data(pTFTHandle, Black);
		}
	}


	// 2. Draw dotted grid
	if(State == STATE_SPEED)
	{
		for(int i = 100; i <= 300; i = i + 50)
		{
			for(int j = 20; j <= 215; j = j + 5)
			{
				TFT_Pixel(pTFTHandle, i, j, White);
			}
		}


		for(int j = 20; j <= 200; j = j + 30)
		{
			for(int i = 50; i <= 310; i = i + 5)
			{
				TFT_Pixel(pTFTHandle, i, j, White);
			}
		}
	}

	else if( (State == STATE_POSITION) || (State == STATE_POSITION_TRACKING) )
	{
		for(int i = 100; i <= 300; i = i + 50)
		{
			for(int j = 20; j <= 215; j = j + 5)
			{
				TFT_Pixel(pTFTHandle, i, j, White);
			}
		}


		for(int j = 20; j <= 176; j = j + 39)
		{
			for(int i = 50; i <= 310; i = i + 5)
			{
				TFT_Pixel(pTFTHandle, i, j, White);
			}
		}
	}



	/* Return the Window setting to its Original state */

	// 1. x = 0 ~ 319
	TFT_Write(pTFTHandle, 0x02, 0x0000);
	TFT_Write(pTFTHandle, 0x03, 0x0000);
	TFT_Write(pTFTHandle, 0x04, 0x0001);
	TFT_Write(pTFTHandle, 0x05, 0x003F);


	// 2. y = 0 ~ 239
	TFT_Write(pTFTHandle, 0x06, 0x0000);
	TFT_Write(pTFTHandle, 0x07, 0x0000);
	TFT_Write(pTFTHandle, 0x08, 0x0000);
	TFT_Write(pTFTHandle, 0x09, 0x00EF);

	TFT_Command(pTFTHandle, 0x22);
}


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

