/*
 * tft.h
 *
 *  Created on: 2021. 1. 14.
 *      Author: Ganghyeok Lim
 */

#ifndef TFT_H_
#define TFT_H_

#include "stm32f103xx.h"
#include "common.h"


/* TFT type */
#define TFT1						1
#define TFT2						2

/*=============================================================================================================================*/

/**
  * @brief  TFT LCD Configuration Structure definition
  */
typedef struct
{
	GPIO_TypeDef 		*GPIOx_TFT_Control;

	GPIO_TypeDef 		*GPIOx_TFT_Data;

	uint32_t 			GPIO_Pins_TFT_Control;

	uint32_t			GPIO_Pins_TFT_Data;

	uint32_t			GPIO_Pin_TFT_nRST;

	uint32_t			GPIO_Pin_TFT_nCS;

	uint32_t			GPIO_Pin_TFT_RS;

	uint32_t			GPIO_Pin_TFT_nWR;


} TFT_InitTypeDef;


/**
  * @brief  TFT LCD Handle Structure definition
  */
typedef struct
{
	uint32_t			Instance;

	TFT_InitTypeDef		Init;

	uint8_t				ScreenMode;

	uint8_t				XcharacterLimit;

	uint8_t				YcharacterLimit;

	uint8_t				Xcharacter;

	uint8_t				Ycharacter;

	uint8_t				nextline_flag;

	uint8_t				Xcursor;

	uint8_t				Ycursor;

	uint8_t				cursor_flag;

	uint16_t			cursor;

	uint8_t				underscore_flag;

	uint16_t			underscore;

	uint8_t				outline_flag;

	uint16_t			outline;

	uint16_t			foreground;

	uint16_t			background;

	uint8_t				Kfont_type;


} TFT_HandleTypeDef;


/**************************************************************************************************************
 * 																											  *
 * 												User Macro Definition										  *
 * 									  																		  *
 **************************************************************************************************************/

#define Transparent			0x0821			//   1,   1,   1
#define White				0xFFFF			// 255, 255, 255
#define Black				0x0000			//   0,   0,   0
#define Red					0xF800			// 255,   0,   0
#define Green				0x07E0			//   0, 255,   0
#define Blue				0x001F			//   0,   0, 255
#define Yellow				0xFFE0			// 255, 255,   0
#define Cyan				0x07FF			//   0, 255, 255
#define Magenta				0xF81F			// 255,   0, 255
#define Brown				0xA145			// 160,  40,  40
#define Khaki				0xF731			// 240, 228, 136
#define Orange				0xFD20			// 255, 164,   0
#define Pink				0xFB56			// 255, 104, 176
#define Silver				0xC618			// 192, 192, 192
#define Violet				0xEC1D			// 232, 128, 232
#define Olive				0x8400			// 128, 128,   0
#define Purple				0x8010			// 128,   0, 128
#define Maroon				0x7800			// 128,   0,   0
#define Navy				0x000F			//   0,   0, 128
#define DarkGreen			0x03E0			//   0, 128,   0
#define DarkCyan			0x03EF			//   0, 128, 128
#define DarkGrey			0x7BEF			// 128, 128, 128
#define LightGrey			0xC618			// 192, 192, 192


/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void TFT_Init(TFT_HandleTypeDef *pTFTHandle);
void TFT_MspInit(TFT_HandleTypeDef *pTFTHandle);
void TFT_Command(TFT_HandleTypeDef *pTFTHandle, uint16_t indexRegister);
void TFT_Data(TFT_HandleTypeDef *pTFTHandle, uint16_t data);
void TFT_Write(TFT_HandleTypeDef *pTFTHandle, uint16_t reg, uint16_t val);
void TFT_Clear_Screen(TFT_HandleTypeDef *pTFTHandle);
void TFT_Color_Screen(TFT_HandleTypeDef *pTFTHandle, uint16_t color);
void TFT_GRAM_Address(TFT_HandleTypeDef *pTFTHandle, uint16_t xPos, uint16_t yPos);
void TFT_xy(TFT_HandleTypeDef *pTFTHandle, uint8_t xChar, uint8_t yChar);
void TFT_Color(TFT_HandleTypeDef *pTFTHandle, uint16_t colorfore, uint16_t colorback);
void TFT_Pixel(TFT_HandleTypeDef *pTFTHandle, uint16_t xPos, uint16_t yPos, uint16_t color);
void TFT_Landscape_mode(TFT_HandleTypeDef *pTFTHandle);
void TFT_Portrait_mode(TFT_HandleTypeDef *pTFTHandle);
void TFT_Cursor(TFT_HandleTypeDef *pTFTHandle, uint16_t cursor_color);
void TFT_Underscore(TFT_HandleTypeDef *pTFTHandle, uint16_t underscore_color);
void TFT_Outline(TFT_HandleTypeDef *pTFTHandle, uint16_t outline_color);
void TFT_String(TFT_HandleTypeDef *pTFTHandle, uint8_t xChar, uint8_t yChar, uint16_t colorfore, uint16_t colorback, uint8_t *str);
void TFT_English(TFT_HandleTypeDef *pTFTHandle, uint8_t code);
void TFT_English_pixel(TFT_HandleTypeDef *pTFTHandle, uint16_t Xpixel, uint16_t Ypixel, uint8_t code);
void TFT_Binary(TFT_HandleTypeDef *pTFTHandle, uint32_t number, uint8_t digit);
void TFT_Unsigned_decimal(TFT_HandleTypeDef *pTFTHandle, uint32_t number, uint8_t zerofill, uint8_t digit);
void TFT_Signed_decimal(TFT_HandleTypeDef *pTFTHandle, int32_t number, uint8_t zerofill, uint8_t digit);
void TFT_Hexadecimal(TFT_HandleTypeDef *pTFTHandle, uint32_t number, uint8_t digit);
void TFT_Unsigned_float(TFT_HandleTypeDef *pTFTHandle, float number, uint8_t integral, uint8_t fractional);
void TFT_Signed_float(TFT_HandleTypeDef *pTFTHandle, float number, uint8_t integral, uint8_t fractional);
void TFT_Line(TFT_HandleTypeDef *pTFTHandle, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void TFT_Rectangle(TFT_HandleTypeDef *pTFTHandle, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void TFT_Block(TFT_HandleTypeDef *pTFTHandle, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint16_t fill);
void TFT_Circle(TFT_HandleTypeDef *pTFTHandle, int16_t x1, int16_t y1, int16_t r, uint16_t color);
void TFT_Sine(TFT_HandleTypeDef *pTFTHandle, int16_t peak, uint8_t mode, uint16_t color);
void TFT_Draw_axis(TFT_HandleTypeDef *pTFTHandle);


#endif /* TFT_H_ */
