/*
 * comm_handle.c
 *
 *  Created on: Oct 31, 2017
 *      Author: KLam
 */
/* Includes -------------------------------------------------------------*/
#include "comm_handle.h"
#include "mem_handle.h"
#include "usbd_cdc_if.h"
#include "thermal_print.h"
#include "sensor_handle.h"
#include "misc.h"
#include "test_mod.h"

/* Variables Definitions ------------------------------------------------*/
const char *SETVAL = "set.\0";
const char *GETVAL = "get.\0";
uint8_t u8TxBuf[TX_BUF_SIZE];
char cLogBuf[LOG_PAGES][LOG_BUF_SIZE];
uint8_t u8LogSeq;
LOG_DATA_TYPE DebugLog[LOG_PAGES];
LOG_DATA_TYPE *CurrLogPtr;
//uint8_t u8RxBuf[RX_BUF_SIZE];
//uint8_t u8ReadRxBuf[RX_BUF_SIZE];
/* Function Definitions -------------------------------------------------*/

/**
  * @brief  Perform Log Buffer Initialization
  * @param	None
  * @param	None
  * @retval None
  */
void LogBuf_Init(void){
	uint8_t i;
	for(i=0;i<LOG_PAGES;i++){
		DebugLog[i].cdataptr = &cLogBuf[i][0];
		DebugLog[i].u16Length = 0;
		memset(DebugLog[i].cdataptr,0,LOG_BUF_SIZE);
	}
	u8LogSeq = 0;
	CurrLogPtr = &DebugLog[u8LogSeq];
}


/**
  * @brief  Perform receive data decode
  * @param	u8RcvPtr: data start address
  * @param	u32Length: data length address
  * @retval 0 success or 1 fail
  */
uint32_t Data_Handle(uint8_t* u8RcvPtr, uint32_t u32Length){
	uint8_t u8Cmd;
	int32_t s32Value;
	uint8_t u16Length;
	uint32_t u32HandleLength;
	uint8_t* ptr;
	uint32_t i;
	char *z;
	/*
	if(u32Length <1024){
		//this call will handle n bytes data
		z = strchr((char*)u8RcvPtr,'\n');
		u32HandleLength = z-(char*)u8RcvPtr;
		u32HandleLength++;
	}
	else{
		u32HandleLength = u32Length;
	}*/
	if(!(strncmp((char*)u8RcvPtr, SETVAL, 4))){
		z = strchr((char*)u8RcvPtr,'\n');
		u32HandleLength = z-(char*)u8RcvPtr;
		u32HandleLength++;
		//Set Value receive
		sscanf((char*)(u8RcvPtr+4),"%d",(int*)(&u8Cmd));
		z = strchr((char*)u8RcvPtr,':');
		sscanf(++z,"%i",(int*)(&s32Value));
		switch (u8Cmd){
		case 25:{
			if(s32Value == 0){
				HAL_GPIO_WritePin(PH_STR_1_GPIO_Port, PH_STR_1_Pin, GPIO_PIN_RESET);
			}
			else{
				HAL_GPIO_WritePin(PH_STR_1_GPIO_Port, PH_STR_1_Pin, GPIO_PIN_SET);
			}
		}break;
		case 10:{
			if(HeadUpData.u8State == HEAD_OPEN){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Head Open\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				break;
			}
			if(EngineData.u16State & ENGINE_READY){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Task start\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				EngineData.u16State &= ~ENGINE_READY;
				EngineData.u16State |= ENGINE_START;
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Task handler busy\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}break;
		case 11:{
			if(HeadUpData.u8State == HEAD_OPEN){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Head Open\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				break;
			}
			if(EngineData.u16State & ENGINE_READY){
				EngineData.u16State &= ~ENGINE_READY;
				EngineData.u16State |= ENGINE_START;
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Task handler busy\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}//no break here
		case 12:{
			if(HeadUpData.u8State == HEAD_OPEN){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Head Open\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				break;
			}
			if((s32Value < 7) && (s32Value > -7)){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"%i steps too short, it shall feed >=8 steps\r\n", (int)s32Value);
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}else if((s32Value > 600000) || (s32Value <-600000)){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"%i steps too much, it shall feed <=600000 steps\r\n", (int)s32Value);
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
			else{
				if(s32Value>0){
					PrintTask.u32ForFeedSteps = (uint32_t)s32Value;
					PrintTask.u8SetSpd = FEED_6_IPS;
				}
				else{
					PrintTask.u32BackFeedSteps = (uint32_t)(0-s32Value);
					PrintTask.u8SetSpd = FEED_4_IPS;
				}
				if(EngineData.u16State & ENGINE_READY){
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Set Feed Parameter as %i steps\r\n", (int)s32Value);
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
				}
				else{
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Feed %i steps\r\n", (int)s32Value);
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
				}
			}
		}break;
		case 13:{
			if(HeadUpData.u8State == HEAD_OPEN){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Head Open\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				break;
			}
			if((s32Value>=100) && (s32Value<200)){
				u8MTRCurrentLimitPWM = (uint8_t)s32Value;
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"MT_CURREF PWM Limit= %d\r\n",u8MTRCurrentLimitPWM);
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Value out of range, 100<PWM<200\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}break;
		case 14:{
			if(HeadUpData.u8State == HEAD_OPEN){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Head Open\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				break;
			}
			if((s32Value>=0) && (s32Value<=5000)){
				if(EngineData.u16State & ENGINE_READY){
					u8PrintTestMod = FLASH_LABEL;
					Label_Loader(u8PrintTestMod, (uint32_t)s32Value);
					EngineData.u16State &= ~ENGINE_READY;
					EngineData.u16State |= ENGINE_START;
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Print %d labels from flash\r\n",(int)s32Value);
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
				}else{
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Task handler busy\r\n");
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
				}
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Value out of range, 0<Lable<500\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}break;
		case 19:{
			if(HeadUpData.u8State == HEAD_OPEN){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Head Open\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				break;
			}
			if((s32Value>=0) && (s32Value<=5000)){
				if(EngineData.u16State & ENGINE_READY){
					u8PrintTestMod = LENGTH_TEST;
					Label_Loader(u8PrintTestMod, (uint32_t)s32Value);
					EngineData.u16State &= ~ENGINE_READY;
					EngineData.u16State |= ENGINE_START;
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Print %d Length test labels from flash\r\n",(int)s32Value);
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
				}else{
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Task handler busy\r\n");
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
				}
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Value out of range, 0<Lable<500\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}break;

		case 15:{
			if((s32Value>=0) && (s32Value<=30)){
				PrintTask.u8SetDensity = (uint8_t)s32Value;
				//EngineData.u8Darkness = PrintTask.u8SetDensity;
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Darkness set to: %d\r\n",PrintTask.u8SetDensity);
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Value out of range, 0 <= Darkness <= 30\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}break;
		case 16:{
			if(HeadUpData.u8State == HEAD_OPEN){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Head Open\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				break;
			}
			if(EngineData.u16State & ENGINE_READY){
				u8PrintTestMod = SENSOR_CURVE;
				Label_Loader(u8PrintTestMod, 1);
				EngineData.u16State &= ~ENGINE_READY;
				EngineData.u16State |= ENGINE_START;
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Print %d sensor labels\r\n", (int)s32Value);
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Task handler busy\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}break;
		case 17:{
			if(HeadUpData.u8State == HEAD_OPEN){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Head Open\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				break;
			}
			if(EngineData.u16State & ENGINE_READY){
				u8PrintTestMod = PIXEL_TEST;
				Label_Loader(u8PrintTestMod, 1);
				EngineData.u16State &= ~ENGINE_READY;
				EngineData.u16State |= ENGINE_START;
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Print %d pixel test labels\r\n", (int)s32Value);
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Task handler busy\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}break;
		case 18:{
			if(HeadUpData.u8State == HEAD_OPEN){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Head Open\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				break;
			}
			if(EngineData.u16State & ENGINE_READY){
				u8PrintTestMod = FENCE_TEST;
				Label_Loader(u8PrintTestMod, 1);
				EngineData.u16State &= ~ENGINE_READY;
				EngineData.u16State |= ENGINE_START;
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Print %d fence test labels\r\n", (int)s32Value);
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Task handler busy\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}break;
		case 0:{
			Stop_MTR();
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"Skip Current Task\r\n");
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
		}break;

		case 9:{
			if(s32Value==0){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Watch Dog OFF\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				DebugSwitch &= ~PH_WD_ON;
			}
			else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Watch Dog ON\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				DebugSwitch |= PH_WD_ON;
			}
		}break;
		case 8:{
			if(s32Value==0){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"PH SPI OFF.\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				DebugSwitch &= ~PH_SPI_ON;
			}
			else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"PH SPI ON.\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				DebugSwitch |= PH_SPI_ON;
			}
		}break;
		case 20:{
			if(HeadUpData.u8State == HEAD_OPEN){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Head Open\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				break;
			}
			if(EngineData.u16State & ENGINE_READY){
				u16EmitterIntensity = 150;
				SensorData.u8EMIT_PWM = u16EmitterIntensity;
				SensorData.u8CalibState |= (CALIB_ENABLE|CALIB_WITH_Z4000D);
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Start Calib with Z4000D\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Task handler busy\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}break;
		case 21:{
			if(HeadUpData.u8State == HEAD_OPEN){
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Head Open\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				break;
			}
			if(EngineData.u16State & ENGINE_READY){
				SensorData.u8CalibState |= (CALIB_ENABLE|CALIB_WITH_SWATCH);
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Start Calib with Swatch\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Task handler busy\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}break;
		case 22:{
			if((s32Value>=0) && (s32Value<=200)){
				SensorData.u8EMIT_PWM = (uint8_t)s32Value;
				GAP_Sensor_Switch(1);
				__HAL_TIM_SET_COMPARE(&htimSensor, TIM_CHANNEL_EMIT, SensorData.u8EMIT_PWM);
				if (HAL_TIM_PWM_Start(&htimSensor, TIM_CHANNEL_EMIT) != HAL_OK)
				{
					/* PWM generation Error */
					Error_Handler();
				}
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"EMITTER PWM = %d\r\n",SensorData.u8EMIT_PWM);
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Value out of range, 0<PWM<200\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}

		}break;
		case 23:{
			if((s32Value>=0) && (s32Value<=200)){
				EngineData.u8RefcurrPWM = (uint8_t)s32Value;
				__HAL_TIM_SET_COMPARE(&htimMTRCURREF, TIM_CHANNEL_MTRCURREF, EngineData.u8RefcurrPWM);
				if (HAL_TIM_PWM_Start(&htimMTRCURREF, TIM_CHANNEL_MTRCURREF) != HAL_OK)
				{
					/* PWM generation Error */
					Error_Handler();
				}
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"MT_CURREF PWM = %d\r\n",EngineData.u8RefcurrPWM);
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Value out of range, 0<PWM<200\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}break;
		case 41:{
			if(EngineData.u16State & ENGINE_READY){
				ptr = (uint8_t *)(SDRAM_BANK_ADDR);
				for(i=0;i<0x800000;i++){
					*ptr = 0xaa;
					ptr++;
				}
				ptr = (uint8_t *)(SDRAM_BANK_ADDR);
				for(i=0;i<0x800000;i++){
					if(*ptr != 0xaa){
						break;
					}
					ptr++;
				}
				if(i<0x800000){
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Not Match:%d\r\n", (int)i);
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
				}else{
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Match\r\n");
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
				}
			}else{
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"busy\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
			}
		}break;
		default:{
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"No Valid, SETCMD=%d,Val=%i\r\n", u8Cmd, (int)s32Value);
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
		}break;
		}//end of switch

	}
	else if(!(strncmp((char*)u8RcvPtr, GETVAL, 4))){
		//Get Value receive;
		z = strchr((char*)u8RcvPtr,'\n');
		u32HandleLength = z-(char*)u8RcvPtr;
		u32HandleLength++;
		sscanf((char*)(u8RcvPtr+4),"%d",(int*)(&u8Cmd));
		switch (u8Cmd){
		case 1:{
			//sprintf((char*)u8TxBuf, "GAP:%d BLK:%d P24V:%d PHTEMP:%d HEADOPEN:%d INTEMP:%d\r\n",
			//		u32ADFiltered[ADQ_GAP],
			//		u32ADFiltered[ADQ_BLK],
			//		u32ADFiltered[ADQ_P24V],
			//		u32ADFiltered[ADQ_PHTEMP],
			//		u32ADFiltered[ADQ_HEADOPEN],
			//		u32ADFiltered[ADQ_INTEMP]);
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"GAP:%d BLK:%d P24V:%d PHTEMP:%d HEADOPEN:%d INTEMP:%d\r\n",
								(int)u32ADFiltered[ADQ_GAP],
								(int)u32ADFiltered[ADQ_BLK],
								(int)u32ADFiltered[ADQ_P24V],
								(int)u32ADFiltered[ADQ_PHTEMP],
								(int)u32ADFiltered[ADQ_HEADOPEN],
								(int)u32ADFiltered[ADQ_INTEMP]);
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
		}break;
		case 2:{
			//sprintf((char*)u8TxBuf, "EMITT_PWM=%d, MT_CURREF_PWM=%d, MT_CUR_LIMIT= %d \r\n",
			//		(int)SensorData.u8EMIT_PWM,
			//		(int)EngineData.u8RefcurrPWM,
			//		(int)u8MTRCurrentLimitPWM);
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"EMITT_PWM=%d, MT_CURREF_PWM=%d, MT_CUR_LIMIT= %d \r\n",
						(int)SensorData.u8EMIT_PWM,
						(int)EngineData.u8RefcurrPWM,
						(int)u8MTRCurrentLimitPWM);
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
		}break;
		case 10:{
			//sprintf((char*)u8TxBuf,"PrintTask.Status=0x0%x\r\n",PrintTask.u16Status);
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"Engine State=0x0%x\r\n",EngineData.u16State);
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
		}break;
		case 15:{
			//sprintf((char*)u8TxBuf,"Darkness=%d\r\n",PrintTask.u8SetDensity);
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"Darkness=%d\r\n",EngineData.u8Darkness);
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
		}break;
		case 20:{
			//sprintf((char*)u8TxBuf, "CPU Temperature = %dC\r\n", MCU_Temp_Calc(&u32ADFiltered[ADQ_INTEMP]));
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr, "CPU Temperature = %dC\r\n", (int)(MCU_Temp_Calc(&u32ADFiltered[ADQ_INTEMP])));
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
		}break;
		case 30:{
			//sprintf((char*)u8TxBuf, "PH Temperature = %dC\r\n", EngineData.u8PHTempC);
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr, "PH Temperature = %dC\r\n", EngineData.u8PHTempC);
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
		}break;
		default:{
			//sprintf((char*)u8TxBuf, "No Valid, GETCMD=%d\r\n", u8Cmd);
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr, "No Valid, GETCMD=%d\r\n", u8Cmd);
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
		}break;
		}
	}else if((*u8RcvPtr == 'B') && (*(u8RcvPtr+1) == 'M')){
		u32HandleLength = BMP_Loading(u8RcvPtr, u32Length);
	}
	else{
		//CDC_Transmit_FS(u8RcvPtr,u32Length);
		//HAL_Delay(500);
		//sprintf((char*)(u8RcvPtr+u32Length),"\0");
		z = strchr((char*)u8RcvPtr,'\n');
		u32HandleLength = z-(char*)u8RcvPtr;
		u32HandleLength++;
		if(*u8RcvPtr == '0'){
			PrintTask_Init();
			Stop_MTR();
			//sprintf((char*)u8TxBuf, "Emergency Stop\r\n");
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr, "Emergency Stop\r\n");
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
		}
		else{
			//sprintf((char*)u8TxBuf,"The command is not recognized. Please check again.\r\n");
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr, "Pack Length is %d\r\n", (int)u32HandleLength);
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr, "The command is not recognized. Please check again.\r\n");
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
		}
	}
	//CDC_Transmit_FS(u8TxBuf,strlen((char*)u8TxBuf));
	return u32HandleLength;
}

/**
  * @brief  Perform USB packet decode, called in main()
  * @param	None
  * @retval None
  */
void USB_Receive(void){
	//uint8_t i,j;
	uint8_t temp;
	uint16_t u16Length;
	uint32_t u32HandledLength;
	if(SDRAMBuf[u8RdBufSeq].u8State == RAM_WR_DONE){
		SDRAMBuf[u8RdBufSeq].u8State = RAM_READING;
#ifdef DEBUG_IS_ON
		if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
			sprintf((*CurrLogPtr).cdataptr, "Job Received, Length:%d\r\n",(int)SDRAMBuf[u8RdBufSeq].u32Length);
			u16Length = strlen((*CurrLogPtr).cdataptr);
			(*CurrLogPtr).u16Length += u16Length;
			(*CurrLogPtr).cdataptr += u16Length;
		}
#endif
	}
	if(SDRAMBuf[u8RdBufSeq].u8State == RAM_READING){
		if(SDRAMBuf[u8RdBufSeq].u32Length){
			u32HandledLength = Data_Handle((uint8_t*)(SDRAMBuf[u8RdBufSeq].cRAMPtr), SDRAMBuf[u8RdBufSeq].u32Length);
			SDRAMBuf[u8RdBufSeq].u32Length -= u32HandledLength;
			SDRAMBuf[u8RdBufSeq].cRAMPtr += u32HandledLength;
		}
		if(SDRAMBuf[u8RdBufSeq].u32Length == 0){
			SDRAMBuf[u8RdBufSeq].u8State = RAM_IDLE;
			u8RdBufSeq++;
			u8RdBufSeq &= (SDRAM_BUF_PAGES-1);
		}
	}
		if((*CurrLogPtr).u16Length){
			temp = u8LogSeq;
			u8LogSeq++;
			u8LogSeq &= (uint8_t)(LOG_PAGES-1);
			CurrLogPtr = &DebugLog[u8LogSeq];
			CDC_Transmit_FS((uint8_t*)(&cLogBuf[temp][0]),DebugLog[temp].u16Length);
			DebugLog[temp].cdataptr = &(cLogBuf[temp][0]);
			DebugLog[temp].u16Length = 0;
		}
	/*
	if(u8MemStatus & MEM_PAGE1_WR_DONE){
		u8MemStatus &= ~MEM_PAGE1_WR_DONE;
		u8MemStatus |= MEM_PAGE1_READING;
		//handle page1 data;
		Data_Handle((uint8_t*)SDRAM_PAGE1_ADDR,u32Page1RcvLength);
		//handle page1 finish;
		u8MemStatus &= ~MEM_PAGE1_READING;
		u8MemStatus |= MEM_PAGE1_RD_DONE;
	}
	else if(u8MemStatus & MEM_PAGE2_WR_DONE){
		u8MemStatus &= ~MEM_PAGE2_WR_DONE;
		u8MemStatus |= MEM_PAGE2_READING;
		//handle page2 data;
		Data_Handle((uint8_t*)SDRAM_PAGE2_ADDR,u32Page2RcvLength);
		//handle page2 finish;
		u8MemStatus &= ~MEM_PAGE2_READING;
		u8MemStatus |= MEM_PAGE2_RD_DONE;
	}*/
}

/**
  * @brief  Perform Data sendout
  * @param	None
  * @retval None
  */
void Debug_Msg_Out(char* MsgPtr){
	CDC_Transmit_FS((uint8_t*)MsgPtr,strlen((char*)MsgPtr));
	//while(huart3.gState != HAL_UART_STATE_READY);
	//HAL_UART_Transmit_IT(&huart3, MsgPtr,strlen((char*)MsgPtr));
}
