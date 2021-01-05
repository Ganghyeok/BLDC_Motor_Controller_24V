/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "main.h"


int main(void)
{
	// 1. System Clock configuration to 72MHz
	SystemClock_Config(SYSCLK_FREQ_72MHZ);

	Delay_ms(3000);

	// 2. Clear All members of Handle structures to 0
	MemsetHandleStructure();

	// 3. Initialize peripherals
	Button_Init();				// Initialize peripherals related to Button
	BLDC1_Init();				// Initialize peripherals related to BLDC motor
	TIM6_Init();				// Initialize TIM6 to generate interrupt of 1ms period
	UART2_Init();
	Delay_ms(10);

	// 4. Start PWM for UB, VB, WB
	StartTimerPwm(&BLDC1Handle);
	Delay_ms(10);

	// 5. Disable All PWM channels
	DisableTimerPwmChannel(&BLDC1Handle);
	Delay_ms(10);

	// 6. Set Desired PWM duty to 80%
	BLDC_SET_ROTATION_DIRECTION(&BLDC1Handle, CW);
	BLDC_SET_REFERENCE_DUTY(80);

	char HallCountStr[10] = {0,};

	while(1)
	{
		sprintf(HallCountStr, "%.2lf", (double)BLDC1Handle.Position);
		strcat(HallCountStr, "[deg]\n");
		USART_Transmit(&UART2Handle, (uint8_t*)HallCountStr, strlen((char*)HallCountStr));
		Delay_ms(1000);

		// 1. Check the START/STOP Button is pressed
		if(ButtonFlag == FLAG_SET)
		{
			if(BLDC1Handle.MotorState == STOP)
			{
				// 1. Change MotorState from STOP to START
				BLDC1Handle.MotorState = START;

				// 2. Enable EXTI of Hall sensor
				ENABLE_HALLSENSOR_EXTI();

				// 3. Drive motor to trigger EXTI
				SetPwmDuty(&BLDC1Handle, 10);

				// 4. Charge Bootstrap Capacitor for 10ms before Drive BLDC motor
				BLDC_BootstrapCap_Charge(&BLDC1Handle);

				// 5. Detect current HallPhase location
				BLDC1Handle.HallPhase = (READ_BIT(GPIOC->IDR, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8)) >> 6U;

				BLDC_FIND_OLD_HALLPHASE(&BLDC1Handle);

				// 6. Drive BLDC motor according to HallPhase location
				BLDC_Drive(&BLDC1Handle);

				// 7. Increase PWM duty cycle from 5[%] to DutyRef[%]
				for(int duty = 10; duty <= DutyRef; duty += 5)
				{
					SetPwmDuty(&BLDC1Handle, duty);
					Delay_ms(300);
				}
			}

			else if(BLDC1Handle.MotorState == START)
			{
				// 1. Change MotorState from START to STOP
				BLDC1Handle.MotorState = STOP;

				// 2. Decrease PWM duty cycle from DutyRef[%] to 0[%]
				for(int duty = DutyRef; duty >= 0; duty -= 5)
				{
					SetPwmDuty(&BLDC1Handle, duty);
					Delay_ms(300);
				}

				// 3. Wait until the BLDC motor stops
				Delay_ms(100);

				// 4. Disable EXTI of Hall sensor
				DISABLE_HALLSENSOR_EXTI();

				// 5. Clear GPIO pin of Top side(UT, VT, WT)
				GPIO_WritePin(BLDC1Handle.Init.GPIOx_Top, BLDC1Handle.Init.GPIO_Pins_Top, GPIO_PIN_RESET);
			}

			ButtonFlag = FLAG_RESET;
		}
	}
}
