/*
 * test_mod.h
 *
 *  Created on: Nov 10, 2017
 *      Author: KLam
 */

#ifndef TEST_MOD_H_
#define TEST_MOD_H_
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/* Macro Definitions ---------------------------------------------------------*/
#define FLASH_LABEL 	(1)
#define SDRAM_LABEL		(2)
#define SENSOR_CURVE	(3)
#define PH_TEST			(4)
#define PIXEL_TEST		(5)
#define FENCE_TEST		(6)
#define LENGTH_TEST		(7)

/* Exported Variables --------------------------------------------------------*/
extern	uint8_t u8PrintTestMod;
typedef struct{
	uint32_t u32BMPLength;
	uint32_t u32Reserved;
	uint32_t u32ImgOffset;
	uint32_t u32HeadSize;
	uint32_t u32ImgWidth;
	uint32_t u32ImgLength;
}BMP_HEAD_TYPE;

/* Exported Functions --------------------------------------------------------*/
void Fence_Graphic_Calc(void);
void Pixel_Graphic_Calc(void);
void Label_Loader(uint8_t TestMod, uint32_t u32LabelCount);
uint32_t BMP_Loading(uint8_t* u8DataPtr, uint32_t u32Length);
#endif /* TEST_MOD_H_ */
