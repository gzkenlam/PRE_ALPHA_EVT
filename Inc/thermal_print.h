/*
 * thermal_print.h
 *
 *  Created on: Nov 3, 2017
 *      Author: KLam
 */

#ifndef THERMAL_PRINT_H_
#define THERMAL_PRINT_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/* Macro Definitions ---------------------------------------------------------*/
//Sensor Macros
#define	htimMTRCURREF			htim4
#define htimStepper				htim3
#define htimLatch				htim5
#define htimStrobe				htim2
#define hspiPH					hspi1
#define TIM_CHANNEL_MTRCURREF	TIM_CHANNEL_4

//Motor Macros
#define USE_5NK_MOTOR
#define USE_FULL_STEP

#define DISTANCE_GAPTOTEAR	(200)	//28mm
#define DISTANCE_GAPTOHEAD	(130)	//18mm
#define DISTANCE_HEADTOTEAR	(75)
#define MAX_LABEL_LENGTH	(203*6)
#define MAX_PRINT_TEMP		(70)

#ifdef USE_5NK_MOTOR
#define FEED_3_IPS	(3)
#define FEED_4_IPS	(7)
#define	FEED_5_IPS	(12)
#define	FEED_6_IPS	(17)
#define TABLE_SIZE	(FEED_6_IPS+1)
#define MTR_DIR_PIN_VALUE	GPIO_PIN_SET
#else
#define MTR_DIR_PIN_VALUE	GPIO_PIN_RESET
#endif
#define CURR_LIMIT_PWM	136
//PH Macros
#define NTC_TABLE_SIZE (26)


#define PH_BYTE_WIDTH	(104)
#define PH_WORD_WIDTH	(PH_BYTE_WIDTH>>2)
#define	STROBE_COUNT	(4)
#define	LINEBUF_COUNT	(8)

#define STROBE_COUNT_IS_3
#ifdef STROBE_COUNT_IS_3
#define STROBE_0_TIME	(186)
#define	STROBE_1_TIME	(220)
#define PH_SPICLK_MHZ	(8)
#else
#define STROBE_0_TIME	(186)
#define	STROBE_1_TIME	(180)
#define PH_SPICLK_MHZ	(8)
#endif

#define MIN_LATCH_TIME	(((1000/PH_SPICLK_MHZ)*(8*PH_BYTE_WIDTH))/500+10)



/* Exported Variables --------------------------------------------------------*/
//PWM timers
extern	TIM_HandleTypeDef htim4;		//TIM4 Ch1 for GAP Sensor EMITTER ch4 for MTR_CURREF
extern	TIM_HandleTypeDef htim3;		//TIM3 for STEPPER
extern	TIM_HandleTypeDef htim5;		//TIM5 for STROBE SPI data latch time
extern	TIM_HandleTypeDef htim2;		//TIM2 for STROBE length time
extern	SPI_HandleTypeDef hspi1;		//SPI1 for PH SPI

extern	const uint16_t u16StbTempComp[];
extern	const uint16_t u16StbDarkness[];
extern	const uint16_t u16StrobeTable[];
extern	const uint16_t u16StepperAccTableHalf[];
extern	const uint8_t u8MTRCurrLimitTable[];
extern	const uint16_t u16PHTempTblM[];
extern	const uint16_t u16PHTempTblL[];

typedef struct{
	uint32_t u32BackFeedSteps;
	uint32_t u32ForFeedSteps;
	uint32_t u32LabelCounter;
	uint32_t u32LabelLength;
	uint16_t u16LabelWidth;
	uint16_t u16Status;
	uint8_t u8SensorType;
	uint8_t u8SetSpd;
	uint8_t u8SetDensity;
	uint8_t* u8DataHeadPtr;
}PRINT_TASK_TYPE;
extern PRINT_TASK_TYPE PrintTask;
#define TASK_EMPTY 			0x0001
#define TASK_IMAGEPRINT 	0x0004
#define TASK_SENSECURVE		0x0100
#define TASK_PIXELGRAPHIC	0x0200
#define TASK_FENCEGRAPHIC	0x0800

/*//Sensor Type
#define USE_GAP_SENSOR	0x01
#define USE_BLK_SENSOR	0x02
//Print Task Status
#define TASK_READY 		0x0001
#define TASK_DONE		0x0002
#define TASK_IMAGEPRINT	0x0004
#define TASK_ERROR		0x0008
#define TASK_GAPFOUND	0x0010
#define TASK_PRINTON	0x0020
#define TASK_START		0x0040
#define MTR_ON			0x0080
#define TASK_SENSECURVE	0x0100
#define TASK_PIXELGRAPHIC	0x0200
#define TASK_OVERHEAT	0x0400
#define TASK_FENCEGRAPHIC	0x0800*/

typedef struct{
	uint8_t u8PHTempC;
	uint8_t u8Spd;
	uint8_t u8Darkness;
	uint16_t u16State;
	uint16_t u16PrevStepperPeriod;
	uint16_t u16CurrStepperPeriod;
	uint32_t u32StepCount;
	uint32_t u32PrintLineCount;
	uint16_t u16PrintLineWidth;
	uint8_t u8RefcurrPWM;
	uint8_t u8StepPhase;
	uint8_t u8StrobeSeq;
	uint8_t u8LineBufPtr;
	uint8_t* u8DataPtr;
	uint16_t u16StrobeTime[STROBE_COUNT];
	uint16_t u16LatchTime[STROBE_COUNT];
	uint32_t u32LineBuf[LINEBUF_COUNT][PH_WORD_WIDTH];
	uint32_t u32StrobeData[STROBE_COUNT][PH_WORD_WIDTH];
}ENGINE_DATA_TYPE;
extern ENGINE_DATA_TYPE EngineData;
//Engine State
#define ENGINE_READY 		(0x0001)
#define ENGINE_DONE			(0x0002)
#define ENGINE_IMAGEPRINT	(0x0004)
#define ENGINE_ERROR		(0x0008)
#define ENGINE_GAPFOUND		(0x0010)
#define ENGINE_PRINTON		(0x0020)
#define ENGINE_START		(0x0040)
#define ENGINE_MTRON		(0x0080)
#define ENGINE_SENSECURVE	(0x0100)
#define ENGINE_PIXELGRAPHIC	(0x0200)
#define ENGINE_OVERHEAT		(0x0400)
#define ENGINE_FENCEGRAPHIC	(0x0800)
#define ENGINE_USEGAP		(0x1000)

extern uint8_t u8MTRCurrentLimitPWM;
extern uint32_t u32DynamicGraphic[26];

/* Exported Functions --------------------------------------------------------*/
void PrintEng_Init(void);
void EngineData_Init(void);
void Start_MTR(GPIO_PinState PinState);
void Stop_MTR(void);
uint8_t PH_Temp_Calc(uint32_t* u32ADCPtr);
uint8_t PrintTask_Handler(void);
void Stepper_FullHalf_ISR(void);
void PrintTask_Init(void);
void Strobe_ISR(void);
void Latch_ISR(void);

#endif /* THERMAL_PRINT_H_ */
