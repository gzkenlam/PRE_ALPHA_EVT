/*
 * sensor_handle.h
 *
 *  Created on: Nov 13, 2017
 *      Author: KLam
 */

#ifndef SENSOR_HANDLE_H_
#define SENSOR_HANDLE_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/* Macro Definitions ---------------------------------------------------------*/
//Sensor Macros
#define htimSensor				htim4
#define	TIM_CHANNEL_EMIT		TIM_CHANNEL_1
#define MAX_PWM					200
#define EMIT_PWM_DEFAULT		110


/* Exported Variables --------------------------------------------------------*/
//PWM timers
extern	TIM_HandleTypeDef htim4;		//TIM4 Ch1 for GAP Sensor EMITTER ch4 for MTR_CURREF

typedef struct{
	uint16_t u16Max[8];
	uint16_t u16Min[8];
	uint16_t u16ADThreshold;
	uint16_t u16PrevDelta;
	uint8_t	u8EMIT_PWM;
	uint8_t u8PosDetect;
	uint8_t u8CalibState;
}SENSER_DATA_TYPE;
extern SENSER_DATA_TYPE SensorData;
#define CALIB_ENABLE 0x80		//for CalibState
#define CALIB_WITH_Z4000D	0x40
#define CALIB_WITH_SWATCH	0x20

#define SWATCH_INTENSITY_LOW 800
#define SWATCH_INTENSITY_HIGH 900
#define Z4000D_INTENSITY_LOW 1250
#define Z4000D_INTENSITY_HIGH 1400

typedef struct{
	uint16_t u16Max[8];
	uint16_t u16Min[8];
	uint16_t u16ADTreshold;
	uint8_t u8State;
	uint8_t u8PrevState;
}HEADOPEN_DATA_TYPE;
extern HEADOPEN_DATA_TYPE HeadUpData;
#define HEAD_CLOSE	0x01
#define HEAD_OPEN 	0x02

extern uint16_t u16EmitterIntensity;

/* Exported Functions --------------------------------------------------------*/
void GAP_Sensor_Switch(uint8_t u8OnOff);
void SensorData_Init(void);
uint8_t GAP_Sense_Detect(uint32_t* u32ADPtr);
void Sensor_Curve_Calc(uint32_t* u32ADPtr, uint8_t* u8GraphicLine);
void Head_Open_Detection(uint32_t* u32AdcPtr);
void HeadUpData_Init(void);
void Calib_Intensity(void);
void Label_Calib(void);

#endif /* SENSOR_HANDLE_H_ */
