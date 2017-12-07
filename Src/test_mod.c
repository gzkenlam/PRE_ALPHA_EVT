/*
 * test_mod.c
 *
 *  Created on: Nov 10, 2017
 *      Author: KLam
 */
#include "test_mod.h"
#include "thermal_print.h"
#include "sensor_handle.h"
#include "mem_handle.h"
#include "comm_handle.h"

uint8_t u8PrintTestMod = 0;
/**
  * @brief  Perform update value to Print Task
  * @param	None
  * @retval None
  */
void Label_Loader(uint8_t TestMod, uint32_t u32LabelCount){
	//static uint32_t u32PrevLabelCounter;
	switch(TestMod){
	case FLASH_LABEL:{
		//Print internal save label from flash
		if(u32LabelCount > 0 ){
			PrintTask.u32LabelCounter += u32LabelCount;
			if(!(EngineData.u16State & ENGINE_MTRON)){
				//insert backfeed for 1st page
				PrintTask.u32BackFeedSteps = DISTANCE_HEADTOTEAR;
				PrintTask.u8SetSpd = FEED_3_IPS;
				//PrintTask.u8SensorType = 0;
				EngineData.u16State &= ~ENGINE_USEGAP;
			}
		}else{
			if((PrintTask.u32LabelLength == 0) && (PrintTask.u32LabelCounter>0)){
				//Task has been loaded to EngineData, and there is still task to load new data
				PrintTask.u8DataHeadPtr = (uint8_t*)(u8Image);
				PrintTask.u32LabelLength = 1200;
				PrintTask.u16LabelWidth = 832;
				//PrintTask.u8SensorType = USE_GAP_SENSOR;
				PrintTask.u16Status |= TASK_IMAGEPRINT;
				PrintTask.u16Status &= ~TASK_EMPTY;
			}
		}
		if(!PrintTask.u32LabelCounter){
			u8PrintTestMod = 0;
		}
	}break;
	case PIXEL_TEST:{
		if(u32LabelCount > 0 ){
			PrintTask.u32LabelCounter += u32LabelCount;
			if(!(EngineData.u16State & ENGINE_MTRON)){
				//insert backfeed for 1st page
				PrintTask.u32BackFeedSteps = DISTANCE_HEADTOTEAR;
				PrintTask.u8SetSpd = FEED_3_IPS;
				//PrintTask.u8SensorType = 0;
				EngineData.u16State &= ~ENGINE_USEGAP;
			}
		}else{
			//u32PrevLabelCounter = PrintTask.u32LabelCounter;
			if((PrintTask.u32LabelLength == 0) && (PrintTask.u32LabelCounter>0)){
				//Task has been loaded to EngineData, and there is still task to load new data
				PrintTask.u8DataHeadPtr = (uint8_t*)(u32DynamicGraphic);
				PrintTask.u32LabelLength = 2000;
				PrintTask.u16LabelWidth = 832;
				//PrintTask.u8SensorType = USE_GAP_SENSOR;
				EngineData.u16State &= ~ENGINE_IMAGEPRINT;
				EngineData.u16State |= ENGINE_PIXELGRAPHIC;
			}
		}
		if(!PrintTask.u32LabelCounter){
			u8PrintTestMod = 0;
		}
	}break;
	case FENCE_TEST:{
		if(u32LabelCount > 0 ){
			PrintTask.u32LabelCounter += u32LabelCount;
			if(!(EngineData.u16State & ENGINE_MTRON)){
				//insert backfeed for 1st page
				PrintTask.u32BackFeedSteps = DISTANCE_HEADTOTEAR;
				PrintTask.u8SetSpd = FEED_3_IPS;
				//PrintTask.u8SensorType = 0;
				EngineData.u16State &= ~ENGINE_USEGAP;
			}
		}else{
			//u32PrevLabelCounter = PrintTask.u32LabelCounter;
			if((PrintTask.u32LabelLength == 0) && (PrintTask.u32LabelCounter>0)){
				//Task has been loaded to EngineData, and there is still task to load new data
				PrintTask.u8DataHeadPtr = (uint8_t*)(u32DynamicGraphic);
				PrintTask.u32LabelLength = 2000;
				PrintTask.u16LabelWidth = 832;
				//PrintTask.u8SensorType = USE_GAP_SENSOR;
				EngineData.u16State &= ~ENGINE_IMAGEPRINT;
				EngineData.u16State	|= ENGINE_FENCEGRAPHIC;
			}
		}
		if(!PrintTask.u32LabelCounter){
			u8PrintTestMod = 0;
		}
	}break;
	case SENSOR_CURVE:{
		if(u32LabelCount > 0 ){
			PrintTask.u32LabelCounter += u32LabelCount;
			//u32PrevLabelCounter = PrintTask.u32LabelCounter;
			if((PrintTask.u32LabelLength == 0) && (PrintTask.u32LabelCounter>0)){
				//Task has been loaded to EngineData, and there is still task to load new data
				PrintTask.u8DataHeadPtr = (uint8_t*)(u32DynamicGraphic);
				PrintTask.u32LabelLength = 2000;
				PrintTask.u16LabelWidth = 832;
				//PrintTask.u8SensorType = USE_GAP_SENSOR;
				EngineData.u16State &= ~ENGINE_IMAGEPRINT;
				EngineData.u16State |= ENGINE_SENSECURVE;
			}
		}
		if(!PrintTask.u32LabelCounter){
			u8PrintTestMod = 0;
		}
	}break;
	default:{

	}break;
	}
}
/**
  * @brief  Perform a Fence Graphic generation
  * @param	None
  * @retval None
  */
void Fence_Graphic_Calc(void){
	uint8_t i;
	static uint8_t u8LineState;
	static uint16_t u16LineCount;
	if(u16LineCount<80){
		u16LineCount++;
	}else{
		u16LineCount = 0;
		u8LineState++;
		u8LineState &= 0x03;
		//every 80 lines, 10mm
		for(i=0;i<PH_WORD_WIDTH;i++){
			if(u8LineState == 1){
				u32DynamicGraphic[i]= 0x11111111;
			}else if(u8LineState == 2){
				u32DynamicGraphic[i]= 0x22222222;
			}else if(u8LineState == 3){
				u32DynamicGraphic[i]= 0x44444444;
			}else{
				u32DynamicGraphic[i]= 0x88888888;
			}
		}
	}
}

/**
  * @brief  Perform a Pixel Graphic generation
  * @param	None
  * @retval None
  */
void Pixel_Graphic_Calc(void){
	uint8_t* pixel;
	static uint16_t u16DotCount = 0;
	if(u16DotCount<812){
		u16DotCount ++;
	}else{
		u16DotCount = 0;
	}
	pixel = (uint8_t *)(&u32DynamicGraphic[0]);
	memset((char*)pixel,0,PH_BYTE_WIDTH);
	pixel[(u16DotCount/8)]= 0x80;
	pixel[(u16DotCount/8)] >>= (u16DotCount%8);
}

/**
  * @brief  Perform a BMP Print Task loading
  * @param	None
  * @retval None
  */
uint32_t BMP_Loading(uint8_t* u8DataPtr, uint32_t u32Length){
	BMP_HEAD_TYPE* BmpHeadPtr;
	uint16_t u16Length;
	uint32_t u32RetVal;
	static uint8_t u8LastLog;
	u32RetVal = 0;
	if((*u8DataPtr == 'B') && (*(u8DataPtr+1) == 'M')){
		BmpHeadPtr = (BMP_HEAD_TYPE *)(u8DataPtr+2);
		if(((*BmpHeadPtr).u32BMPLength == u32Length)
		&& ((*BmpHeadPtr).u32ImgWidth <= (PH_BYTE_WIDTH*8))){
			//a correct BMP detected
			/*if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"BMP, w:%d,l:%d\r\n", (*BmpHeadPtr).u32ImgWidth,(*BmpHeadPtr).u32ImgLength);
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}*/
			if(PrintTask.u16Status & TASK_EMPTY){
				PrintTask.u32LabelCounter = 1;
				PrintTask.u8DataHeadPtr = (uint8_t*)(u8DataPtr+(*BmpHeadPtr).u32ImgOffset);
				PrintTask.u32LabelLength = (*BmpHeadPtr).u32ImgLength;
				PrintTask.u16LabelWidth = (*BmpHeadPtr).u32ImgWidth;
				PrintTask.u16Status &= ~TASK_EMPTY;
				PrintTask.u16Status |= TASK_IMAGEPRINT;
				u32RetVal = u32Length;
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"BMP Task Loaded, W:%d, L:%d\r\n", (int)((*BmpHeadPtr).u32ImgWidth),(int)((*BmpHeadPtr).u32ImgLength));
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				u8LastLog = 1;
			}
			else{
				//Task busy wait for loaded
				if(u8LastLog != 2){
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"BMP Detected, Task Busy, W:%d, L:%d\r\n", (int)((*BmpHeadPtr).u32ImgWidth),(int)((*BmpHeadPtr).u32ImgLength));
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
				}
				u8LastLog = 2;
				u32RetVal = 0;
				return u32RetVal;
			}
			if(EngineData.u16State & ENGINE_READY){
				EngineData.u16State &= ~ENGINE_READY;
				EngineData.u16State |= ENGINE_START;
				if(!(EngineData.u16State & ENGINE_MTRON)){
					//insert backfeed for 1st page
					PrintTask.u32BackFeedSteps = DISTANCE_HEADTOTEAR;
					PrintTask.u8SetSpd = FEED_3_IPS;
					//PrintTask.u8SensorType = 0;
					EngineData.u16State &= ~ENGINE_USEGAP;
				}
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Print BMP label\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				u8LastLog = 3;
			}else{
				//sprintf((char*)u8TxBuf, "Task handler busy\r\n");
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Engine busy\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
				u8LastLog = 4;
			}
		}else{
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"not match, l:%d, w:%d\r\n", (int)((*BmpHeadPtr).u32BMPLength),(int)((*BmpHeadPtr).u32ImgWidth));
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
			u8LastLog = 5;
			u32RetVal = u32Length;
		}
	}
	return u32RetVal;
}
