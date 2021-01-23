/*
 * user_func.c
 *
 *  Created on: Dec 18, 2020
 *      Author: Ganghyeok Lim
 */

#include "main.h"


/* Peripheral Handle Definitions */
TIM_HandleTypeDef 			TIM6Handle;
TIM_HandleTypeDef 			TIM4Handle;
BLDC_HandleTypeDef 			BLDC1Handle;
UART_HandleTypeDef 			UART3Handle;
DMA_HandleTypeDef			DMA1Handle;
TFT_HandleTypeDef			TFT1Handle;
TS_HandleTypeDef			TS1Handle;
SPI_HandleTypeDef			SPI2Handle;

/* Status Flags */
uint8_t State = 			STATE_MENU;

/* Key Count Variables */

int32_t Mode_key = 				0;			// Count of 'MODE' Key
int32_t Up_key = 				0;			// Count of 'UP' Key
int32_t Down_key = 				0;			// Count of 'DOWN' Key
int32_t Start_key = 			0;			// Count of 'START/STOP' Key
int32_t EmergencyStop_key = 	0;			// Count of 'Emergency STOP' Key

/* Strings for UART */
char MotorSpeedStr[6] = 		{0,};
char MotorPositionStr[8] = 		{0,};
char Msg1[50] = 				{0,};

/* Graph variables */
uint16_t x = 					0;
uint16_t y = 					0;
uint16_t x_prv = 				0;
uint16_t y_prv = 				0;
uint8_t GraphDraw_flag = 		FLAG_RESET;
uint8_t GraphClear_flag = 		FLAG_RESET;


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

	/* Init GPIO of EMERGENCY STOP Button */
	GPIOInit.Pin = GPIO_PIN_4;
	GPIOInit.Mode = GPIO_MODE_IT_FALLING;
	GPIOInit.Pull = GPIO_PULLUP;
	GPIO_Init(GPIOA, &GPIOInit);
	NVIC_IRQConfig(IRQ_NO_EXTI4, NVIC_PRIOR_15, ENABLE);
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


				/* Calculate the Pixel corresponding to the Speed value */
				if( (GraphClear_flag == FLAG_RESET) || (GraphDraw_flag == FLAG_RESET) )
				{
					y = (uint16_t)((200. * BLDC1Handle.CurSpeed) / (BLDC1Handle.RefSpeed * 4. / 3.));

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

				count = 0;
			}
		}


		/* If Motor State is POSITION or POSITION_TRACKING */
		else if( (BLDC1Handle.MotorState == MOTOR_STATE_POSITION) || (BLDC1Handle.MotorState == MOTOR_STATE_POSITION_TRACKING) )
		{
			/* Set PWM duty cycle by Position PID calculation */
			BLDC_PositionPID(&BLDC1Handle, 0.001);

			if(count >= 10)
			{
				/* Calculate the Pixel corresponding to the Speed value */
				if( (GraphClear_flag == FLAG_RESET) || (GraphDraw_flag == FLAG_RESET) )
				{
					y = (uint16_t)((200. * BLDC1Handle.CurPosition) / (BLDC1Handle.RefPosition * 4. / 3.));

					x_prv = x;
					y_prv = y;

					x++;

					GraphDraw_flag = FLAG_SET;

					if(x >= 250)
					{
						GraphClear_flag = FLAG_SET;
					}
				}

				count = 0;
			}


//			/* Transmit Motor Position value to PC through UART3 */
//			if(count >= 2)		// Every 2ms
//			{
//
//				if(BLDC1Handle.MotorState == MOTOR_STATE_POSITION)
//				{
//					// To see the case of RefPosition
//					sprintf(Msg1, "%.2lf,%.2lf\n", BLDC1Handle.RefPosition, BLDC1Handle.CurPosition);
//				}
//				else if(BLDC1Handle.MotorState == MOTOR_STATE_POSITION_TRACKING)
//				{
//					// To see the case of TrjCurPosition
//					sprintf(Msg1, "%.2lf,%.2lf\n", BLDC1Handle.TrjCurPosition, BLDC1Handle.CurPosition);
//				}
//
//
//				UART_Transmit_DMA(&UART3Handle, (uint8_t*)Msg1, strlen((char*)Msg1));
//
//				count = 0;
//			}
		}


		count++;
	}
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
		if(Mode_key >= 3)
		{
			Mode_key = 0;
		}
		else
		{
			Mode_key++;
		}
	}

	else if(GPIO_Pin == GPIO_PIN_1)
	{
		Up_key++;
	}

	else if(GPIO_Pin == GPIO_PIN_2)
	{
		Down_key++;
	}

	else if(GPIO_Pin == GPIO_PIN_3)
	{
		if(Start_key >= 1)
		{
			Start_key = 0;
		}
		else
		{
			Start_key++;
		}
	}

	else if(GPIO_Pin == GPIO_PIN_4)
	{
		if(EmergencyStop_key >= 1)
		{
			EmergencyStop_key = 0;
		}
		else
		{
			EmergencyStop_key++;
		}
	}

	UNUSED(GPIO_Pin);
}



/********************************************************************************************************************
 *							Group of functions which belong to main function for increasing Readability				*
 ********************************************************************************************************************/

void State_Menu(void)
{
	// Reset variables (structure members, global variables)
	Reset_All_Variables();

	TFT_Clear_Screen(&TFT1Handle);
	Delay_ms(200);

	TFT_String(&TFT1Handle, 10, 10, White, Black, (uint8_t*)"Option : ");
	Delay_ms(10);

	Mode_key = 0;

	while(1)
	{
		switch (Mode_key)
		{
			/* Option : Speed Mode */
			case 0 :
			{
				TFT_String(&TFT1Handle, 8, 20, White, Black, (uint8_t*)"      Speed Mode      ");
				State = STATE_SPEED;

				break;
			}

			/* Option : Position Mode */
			case 1 :
			{
				TFT_String(&TFT1Handle, 8, 20, White, Black, (uint8_t*)"     Position Mode    ");
				State = STATE_POSITION;

				break;
			}

			/* Option : Position Tracking Mode */
			case 2 :
			{
				TFT_String(&TFT1Handle, 8, 20, White, Black, (uint8_t*)"Position Tracking Mode");
				State = STATE_POSITION_TRACKING;

				break;
			}

			/* Option : End Mode */
			case 3 :
			{
				TFT_String(&TFT1Handle, 8, 20, White, Black, (uint8_t*)"        End Mode      ");
				State = STATE_END;

				break;
			}

			default :
			{
				break;
			}
		}

		Delay_ms(100);

		if(Start_key >= 1)
		{
			Start_key = 0;
			break;
		}
	}
}


void State_Speed(void)
{
	TFT_Clear_Screen(&TFT1Handle);
	Delay_ms(100);
	Mode_key = FLAG_RESET;

	Draw_axis(&TFT1Handle, State);
	TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
	TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
	Delay_ms(100);

	while(1)
	{
		// Display Reference Speed value
		TFT_xy(&TFT1Handle, 31, 20);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.RefSpeed, 4, 1);

		// Display Current Speed value
		TFT_xy(&TFT1Handle, 31, 22);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.CurSpeed, 4, 1);


		if(GraphClear_flag == FLAG_SET)
		{
			// Erase Graph of Current Position
			Clear_Graph(&TFT1Handle);

			x = 0;
			x_prv = 0;
			y_prv = 0;

			GraphClear_flag = FLAG_RESET;

			TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
			TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
		}
		else if(GraphDraw_flag == FLAG_SET)
		{
			// Draw Graph of Current Position
			TFT_Line(&TFT1Handle, 50+x_prv, 215-y_prv, 50+x, 215-y, Magenta);
			GraphDraw_flag = FLAG_RESET;
		}




		if(Start_key >= FLAG_SET)
		{
			Delay_ms(200);		// to Avoid Key chattering

			if(BLDC1Handle.MotorState == MOTOR_STATE_STOP)
			{
				Clear_Graph(&TFT1Handle);
				Delay_ms(10);
				Draw_axis(&TFT1Handle, State);
				TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
				TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");

				// 1. Set Reference Speed
				BLDC_SET_REFERENCE_SPEED(&BLDC1Handle, 500);

				// Draw axis value
				TFT_Color(&TFT1Handle, Cyan, Black);
				TFT_xy(&TFT1Handle, 0, 1);
				TFT_Unsigned_decimal(&TFT1Handle, (uint32_t)(BLDC1Handle.RefSpeed * 4 / 3), 0, 5);
				TFT_xy(&TFT1Handle, 0, 7);
				TFT_Unsigned_decimal(&TFT1Handle, (uint32_t)BLDC1Handle.RefSpeed, 0, 5);
				TFT_xy(&TFT1Handle, 0, 13);
				TFT_Unsigned_decimal(&TFT1Handle, (uint32_t)(BLDC1Handle.RefSpeed * 2 / 3), 0, 5);
				TFT_xy(&TFT1Handle, 0, 20);
				TFT_Unsigned_decimal(&TFT1Handle, (uint32_t)(BLDC1Handle.RefSpeed * 1 / 3), 0, 5);

				// 2. Set PID gain
				BLDC_PID_GAIN_SET(&BLDC1Handle, 0.02, 8, 0);

				// 3. Set Direction of Rotation
				if(BLDC1Handle.RefSpeed >= 0)			BLDC1Handle.RotationDir = CW;
				else if(BLDC1Handle.RefSpeed < 0)		BLDC1Handle.RotationDir = CCW;

				// 4. Set Old HallPhase location based on Current HallPhase
				BLDC_SET_OLD_HALLPHASE(&BLDC1Handle);

				// 5. Charge Bootstrap Capacitor for 10ms
				BLDC_BootstrapCap_Charge(&BLDC1Handle);

				// 6. Enable EXTI of Hall sensor
				ENABLE_HALLSENSOR_EXTI();

				// 7. Trigger EXTI interrupt by SW to Execute 'BLDC_Drive' function. (Top Logic On, Bottom PWM On. But TIM_CCR == 0)
				EXTI->SWIER |= (0x1 << 5);		// The purpose of this line is to trigger EXTI9_5_IRQHandler. So, 5 can be replaced by 6, 7.

				// 8. Reset HallCount value to 0. When EXTI9_5_IRQHandler is triggered, BLDC_Get_Position function increases / dicrease HallCount value by 1
				BLDC1Handle.HallCount = 0;

				// 9. Change MotorState from MOTOR_STATE_STOP to MOTOR_STATE_SPEED
				BLDC1Handle.MotorState = MOTOR_STATE_SPEED;
			}

			else if(BLDC1Handle.MotorState == MOTOR_STATE_SPEED)
			{
				// 1. Set Reference Speed to 0
				BLDC_SET_REFERENCE_SPEED(&BLDC1Handle, 0);

				// 2. Wait until the Motor stops
				while( ((int16_t)BLDC1Handle.CurSpeed) != 0 )
				{
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
	TFT_Clear_Screen(&TFT1Handle);
	Delay_ms(100);
	Mode_key = FLAG_RESET;

	Draw_axis(&TFT1Handle, State);
	TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
	TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
	Delay_ms(100);


	while(1)
	{
		// Display Reference Position value
		TFT_xy(&TFT1Handle, 31, 20);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.RefPosition, 5, 1);

		// Display Current Position value
		TFT_xy(&TFT1Handle, 31, 22);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.CurPosition, 5, 1);


		if(GraphClear_flag == FLAG_SET)
		{
			// Erase Graph of Current Position
			Clear_Graph(&TFT1Handle);

			x = 0;
			x_prv = 0;
			y_prv = 0;

			GraphClear_flag = FLAG_RESET;

			TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
			TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
		}
		else if(GraphDraw_flag == FLAG_SET)
		{
			// Draw Graph of Current Position
			TFT_Line(&TFT1Handle, 50+x_prv, 215-y_prv, 50+x, 215-y, Magenta);
			GraphDraw_flag = FLAG_RESET;
		}



		if(Start_key >= FLAG_SET)
		{
			Delay_ms(200);		// to Avoid Key chattering

			if(BLDC1Handle.MotorState == MOTOR_STATE_STOP)
			{
				Clear_Graph(&TFT1Handle);
				Delay_ms(10);
				Draw_axis(&TFT1Handle, State);
				TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Ref : ");
				TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");

				// 1. Set Reference Position and PID gain
				BLDC_SET_REFERENCE_POSITION(&BLDC1Handle, 36000);

				// Draw axis value
				TFT_Color(&TFT1Handle, Cyan, Black);
				TFT_xy(&TFT1Handle, 0, 1);
				TFT_Unsigned_decimal(&TFT1Handle, (uint32_t)(BLDC1Handle.RefPosition * 4 / 3), 0, 5);
				TFT_xy(&TFT1Handle, 0, 7);
				TFT_Unsigned_decimal(&TFT1Handle, (uint32_t)BLDC1Handle.RefPosition, 0, 5);
				TFT_xy(&TFT1Handle, 0, 13);
				TFT_Unsigned_decimal(&TFT1Handle, (uint32_t)(BLDC1Handle.RefPosition * 2 / 3), 0, 5);
				TFT_xy(&TFT1Handle, 0, 20);
				TFT_Unsigned_decimal(&TFT1Handle, (uint32_t)(BLDC1Handle.RefPosition * 1 / 3), 0, 5);

				// 2. Set PID gain
				BLDC_PID_GAIN_SET(&BLDC1Handle, 15, 0, 0.01);

				// 3. Set Direction of Rotation
				if(BLDC1Handle.RefPosition >= 0)			BLDC1Handle.RotationDir = CW;
				else if(BLDC1Handle.RefPosition < 0)		BLDC1Handle.RotationDir = CCW;

				// 4. Set Old HallPhase location based on Current HallPhase
				BLDC_SET_OLD_HALLPHASE(&BLDC1Handle);

				// 5. Charge Bootstrap Capacitor for 10ms
				BLDC_BootstrapCap_Charge(&BLDC1Handle);

				// 6. Enable EXTI of Hall sensor
				ENABLE_HALLSENSOR_EXTI();

				// 7. Trigger EXTI interrupt by SW to Execute 'BLDC_Drive' function. (Top Logic On, Bottom PWM On. But TIM_CCR == 0)
				EXTI->SWIER |= (0x1 << 5);		// The purpose of this line is to trigger EXTI9_5_IRQHandler. So, 5 can be replaced by 6, 7.

				// 8. Reset HallCount value to 0. When EXTI9_5_IRQHandler is triggered, BLDC_Get_Position function increases / dicrease HallCount value by 1
				BLDC1Handle.HallCount = 0;

				// 9. Change MotorState from MOTOR_STATE_STOP to MOTOR_STATE_POSITION
				BLDC1Handle.MotorState = MOTOR_STATE_POSITION;
			}

			else if(BLDC1Handle.MotorState == MOTOR_STATE_POSITION)
			{
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
	TFT_Clear_Screen(&TFT1Handle);
	Delay_ms(100);
	Mode_key = FLAG_RESET;

	Draw_axis(&TFT1Handle, State);
	TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Trj : ");
	TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
	Delay_ms(100);

	while(1)
	{
		// Display Trajectory Current Position value
		TFT_xy(&TFT1Handle, 31, 20);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.TrjCurPosition, 5, 1);

		// Display Current Position value
		TFT_xy(&TFT1Handle, 31, 22);
		TFT_Signed_float(&TFT1Handle, BLDC1Handle.CurPosition, 5, 1);


		if(GraphClear_flag == FLAG_SET)
		{
			// Erase Graph of Current Position
			Clear_Graph(&TFT1Handle);

			x = 0;
			x_prv = 0;
			y_prv = 0;

			GraphClear_flag = FLAG_RESET;

			TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Trj : ");
			TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");
		}
		else if(GraphDraw_flag == FLAG_SET)
		{
			// Draw Graph of Current Position
			TFT_Line(&TFT1Handle, 50+x_prv, 215-y_prv, 50+x, 215-y, Magenta);
			GraphDraw_flag = FLAG_RESET;
		}


		if(Start_key >= FLAG_SET)
		{
			Delay_ms(200);		// to Avoid Key chattering

			if(BLDC1Handle.MotorState == MOTOR_STATE_STOP)
			{
				Clear_Graph(&TFT1Handle);
				Delay_ms(10);
				Draw_axis(&TFT1Handle, State);
				TFT_String(&TFT1Handle, 25, 20, White, Black, (uint8_t*)"Trj : ");
				TFT_String(&TFT1Handle, 25, 22, White, Black, (uint8_t*)"Cur : ");


				// 1. Set Reference Position and PID gain
				BLDC_SET_REFERENCE_POSITION(&BLDC1Handle, 36000);
				BLDC1Handle.TrjRefMaxSpeed = 7500;
				BLDC1Handle.TrjRefAcceleration = 2000;

				// Draw axis value
				TFT_Color(&TFT1Handle, Cyan, Black);
				TFT_xy(&TFT1Handle, 0, 1);
				TFT_Unsigned_decimal(&TFT1Handle, (uint32_t)(BLDC1Handle.RefPosition * 4 / 3), 0, 5);
				TFT_xy(&TFT1Handle, 0, 7);
				TFT_Unsigned_decimal(&TFT1Handle, (uint32_t)BLDC1Handle.RefPosition, 0, 5);
				TFT_xy(&TFT1Handle, 0, 13);
				TFT_Unsigned_decimal(&TFT1Handle, (uint32_t)(BLDC1Handle.RefPosition * 2 / 3), 0, 5);
				TFT_xy(&TFT1Handle, 0, 20);
				TFT_Unsigned_decimal(&TFT1Handle, (uint32_t)(BLDC1Handle.RefPosition * 1 / 3), 0, 5);

				// 2. Set PID gain
				BLDC_PID_GAIN_SET(&BLDC1Handle, 25, 4, 0);

				// 3. Set Direction of Rotation
				if(BLDC1Handle.RefPosition >= 0)			BLDC1Handle.RotationDir = CW;
				else if(BLDC1Handle.RefPosition < 0)		BLDC1Handle.RotationDir = CCW;

				// 4. Set Old HallPhase location based on Current HallPhase
				BLDC_SET_OLD_HALLPHASE(&BLDC1Handle);

				// 5. Charge Bootstrap Capacitor for 10ms
				BLDC_BootstrapCap_Charge(&BLDC1Handle);

				// 6. Enable EXTI of Hall sensor
				ENABLE_HALLSENSOR_EXTI();

				// 7. Trigger EXTI interrupt by SW to Execute 'BLDC_Drive' function. (Top Logic On, Bottom PWM On. But TIM_CCR == 0)
				EXTI->SWIER |= (0x1 << 5);		// The purpose of this line is to trigger EXTI9_5_IRQHandler. So, 5 can be replaced by 6, 7.

				// 8. Reset HallCount value to 0. When EXTI9_5_IRQHandler is triggered, BLDC_Get_Position function increases / dicrease HallCount value by 1
				BLDC1Handle.HallCount = 0;

				// 9. Change MotorState from MOTOR_STATE_STOP to MOTOR_STATE_POSITION_TRACKING
				BLDC1Handle.MotorState = MOTOR_STATE_POSITION_TRACKING;
			}

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


void State_End(void)
{
	TFT_String(&TFT1Handle, 5, 15, White, Black, (uint8_t*)"End mode is selected");
	Delay_ms(100);

	while(1)
	{
		if(Start_key >= 1)
		{
			State = STATE_MENU;
			Start_key = 0;
			break;
		}
	}
}


void Reset_All_Variables(void)
{
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

	/* Graph variables */
	TFT_Color(&TFT1Handle, White, Black);
	x = 0;
	y = 0;
	x_prv = 0;
	y_prv = 0;
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
	// Draw X-axis
	TFT_Line(pTFTHandle, 50, 216, 310, 216, White);
	TFT_Line(pTFTHandle, 305, 211, 310, 211, White);
	TFT_Line(pTFTHandle, 305, 221, 310, 216, White);
	TFT_Line(pTFTHandle, 100, 216, 100, 220, White);
	TFT_Line(pTFTHandle, 150, 216, 150, 220, White);
	TFT_Line(pTFTHandle, 200, 216, 200, 220, White);
	TFT_Line(pTFTHandle, 250, 216, 250, 220, White);
	TFT_Line(pTFTHandle, 300, 216, 300, 220, White);

	if(state == STATE_SPEED)
	{
		TFT_String(pTFTHandle, 16, 0, White, Blue, (uint8_t *)"Speed Graph");

		TFT_Color(pTFTHandle, Cyan, Black);
		TFT_English_pixel(pTFTHandle, 35, 222, '0');

		TFT_English_pixel(pTFTHandle, 97, 222, '5');

		TFT_English_pixel(pTFTHandle, 143, 222, '1');
		TFT_English_pixel(pTFTHandle, 151, 222, '0');

		TFT_English_pixel(pTFTHandle, 193, 222, '1');
		TFT_English_pixel(pTFTHandle, 201, 222, '5');

		TFT_English_pixel(pTFTHandle, 243, 222, '2');
		TFT_English_pixel(pTFTHandle, 251, 222, '0');

		TFT_English_pixel(pTFTHandle, 293, 222, '2');
		TFT_English_pixel(pTFTHandle, 301, 222, '5');
	}
	else if(state == STATE_POSITION)
	{
		TFT_String(pTFTHandle, 14, 0, White, Blue, (uint8_t *)"Position Graph");

		TFT_Color(pTFTHandle, Cyan, Black);
		TFT_English_pixel(pTFTHandle, 35, 222, '0');

		TFT_English_pixel(pTFTHandle, 89, 222, '0');
		TFT_English_pixel(pTFTHandle, 97, 222, '.');
		TFT_English_pixel(pTFTHandle, 105, 222, '6');

		TFT_English_pixel(pTFTHandle, 139, 222, '1');
		TFT_English_pixel(pTFTHandle, 147, 222, '.');
		TFT_English_pixel(pTFTHandle, 155, 222, '2');

		TFT_English_pixel(pTFTHandle, 189, 222, '1');
		TFT_English_pixel(pTFTHandle, 197, 222, '.');
		TFT_English_pixel(pTFTHandle, 206, 222, '9');

		TFT_English_pixel(pTFTHandle, 239, 222, '2');
		TFT_English_pixel(pTFTHandle, 247, 222, '.');
		TFT_English_pixel(pTFTHandle, 255, 222, '5');
	}
	else if(state == STATE_POSITION_TRACKING)
	{
		TFT_String(pTFTHandle, 10, 0, White, Blue, (uint8_t *)"Position Tracking Graph");

		TFT_Color(pTFTHandle, Cyan, Black);
		TFT_English_pixel(pTFTHandle, 35, 222, '0');

		TFT_English_pixel(pTFTHandle, 89, 222, '0');
		TFT_English_pixel(pTFTHandle, 97, 222, '.');
		TFT_English_pixel(pTFTHandle, 105, 222, '6');

		TFT_English_pixel(pTFTHandle, 139, 222, '1');
		TFT_English_pixel(pTFTHandle, 147, 222, '.');
		TFT_English_pixel(pTFTHandle, 155, 222, '2');

		TFT_English_pixel(pTFTHandle, 189, 222, '1');
		TFT_English_pixel(pTFTHandle, 197, 222, '.');
		TFT_English_pixel(pTFTHandle, 206, 222, '9');

		TFT_English_pixel(pTFTHandle, 239, 222, '2');
		TFT_English_pixel(pTFTHandle, 247, 222, '.');
		TFT_English_pixel(pTFTHandle, 255, 222, '5');
	}

	//	TFT_Color(pTFTHandle, Cyan, Black);
	//	TFT_English_pixel(pTFTHandle, 35, 222, '0');
	//	TFT_English_pixel(pTFTHandle, 97, 222, '1');
	//	TFT_English_pixel(pTFTHandle, 147, 222, '2');
	//	TFT_English_pixel(pTFTHandle, 197, 222, '3');
	//	TFT_English_pixel(pTFTHandle, 247, 222, '4');
	//	TFT_English_pixel(pTFTHandle, 297, 222, '5');
		TFT_Color(pTFTHandle, Magenta, Black);
		TFT_English_pixel(pTFTHandle, 288, 222, '[');
		TFT_English_pixel(pTFTHandle, 296, 222, 's');
		TFT_English_pixel(pTFTHandle, 304, 222, ']');


	// Draw Y-axis
	TFT_Line(pTFTHandle, 49, 215, 49, 5, White);
	TFT_Line(pTFTHandle, 44, 10, 49, 5, White);
	TFT_Line(pTFTHandle, 54, 10, 49, 5, White);
	TFT_Line(pTFTHandle, 45, 165, 49, 165, White);
	TFT_Line(pTFTHandle, 45, 115, 49, 115, White);
	TFT_Line(pTFTHandle, 45, 65, 49, 65, White);
	TFT_Line(pTFTHandle, 45, 15, 49, 15, White);
}


void Draw_Graph(TFT_HandleTypeDef *pTFTHandle)
{

}


void Clear_Graph(TFT_HandleTypeDef *pTFTHandle)
{
	TFT_Write(pTFTHandle, 0x02, 50U >> 8);
	TFT_Write(pTFTHandle, 0x03, 50U & 0x00FF);
	TFT_Write(pTFTHandle, 0x04, 300U >> 8);
	TFT_Write(pTFTHandle, 0x05, 300U & 0x00FF);

	TFT_Write(pTFTHandle, 0x06, 0x0000);
	TFT_Write(pTFTHandle, 0x07, 15U);
	TFT_Write(pTFTHandle, 0x08, 0x0000);
	TFT_Write(pTFTHandle, 0x09, 215U);

	TFT_Command(pTFTHandle, 0x22);

	for(uint16_t i = 0; i < 251; i++)
	{
		for(uint16_t j = 0; j < 201; j++)
		{
			TFT_Data(pTFTHandle, Black);
		}
	}

	/* Reset Content of Address registers to initial value  */
	TFT_Write(pTFTHandle, 0x02, 0x0000);
	TFT_Write(pTFTHandle, 0x03, 0x0000);
	TFT_Write(pTFTHandle, 0x04, 0x0001);
	TFT_Write(pTFTHandle, 0x05, 0x003F);
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

