/*
 * misc.h
 *
 *  Created on: Oct 27, 2017
 *      Author: KLam
 */

#ifndef MISC_H_
#define MISC_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/* Macro Definitions ---------------------------------------------------------*/
#define	TIME_SLOT_COUNTER_BIT_00	(0x0001)
#define	TIME_SLOT_COUNTER_BIT_01	(0x0002)
#define	TIME_SLOT_COUNTER_BIT_02	(0x0004)
#define	TIME_SLOT_COUNTER_BIT_03	(0x0008)
#define	TIME_SLOT_COUNTER_BIT_04	(0x0010)
#define	TIME_SLOT_COUNTER_BIT_05	(0x0020)
#define	TIME_SLOT_COUNTER_BIT_06	(0x0040)
#define	TIME_SLOT_COUNTER_BIT_07	(0x0080)
#define	TIME_SLOT_COUNTER_BIT_08	(0x0100)
#define	TIME_SLOT_COUNTER_BIT_09	(0x0200)
#define	TIME_SLOT_COUNTER_BIT_10	(0x0400)
#define	TIME_SLOT_COUNTER_BIT_11	(0x0800)
#define	TIME_SLOT_COUNTER_BIT_12	(0x1000)
#define	TIME_SLOT_COUNTER_BIT_13	(0x2000)
#define	TIME_SLOT_COUNTER_BIT_14	(0x4000)
#define	TIME_SLOT_COUNTER_BIT_15	(0x8000)

//AD converter
#define ADQ_GAP				(0)
#define ADQ_PHTEMP			(1)
#define	ADQ_HEADOPEN 		(2)
#define	ADQ_P24V			(3)
#define ADQ_INTEMP			(4)
#define ADQ_BLK				(5)

/* Exported Variables  -------------------------------------------------------*/
extern	uint16_t u16TimerSlotCounter;
extern	uint32_t u32ADTemp[3][6];
extern	uint32_t u32ADFiltered[6];
extern	uint8_t u8LEDState;

//Debug only
#ifdef DEBUG_IS_ON
#define PH_WD_ON	(0x0001)
#define PH_SPI_ON		(0x0002)
uint32_t DebugSwitch;
#endif

/* Exported Functions --------------------------------------------------------*/
void System_Tick_ISR(void);
void ADC_Filter_ISR(void);
int32_t MCU_Temp_Calc(uint32_t* u32ADPtr);
void Feed_Key_Sampling(void);
void LED_Rreshing(void);


#endif /* MISC_H_ */
