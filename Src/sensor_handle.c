/*
 * sensor_handle.c
 *
 *  Created on: Nov 13, 2017
 *      Author: KLam
 */
#include "thermal_print.h"
#include "sensor_handle.h"
#include "comm_handle.h"
#include "test_mod.h"
#include "misc.h"
#include <string.h>

uint16_t u16EmitterIntensity = 100;
uint16_t u16HeadUpThreshold = 700;

HEADOPEN_DATA_TYPE HeadUpData;
SENSER_DATA_TYPE SensorData;
/**
  * @brief  Perform SensorData Initial
  * @param	None
  * @retval None
  */
void SensorData_Init(void){
	uint8_t i;
	for(i=0;i<8;i++){
		SensorData.u16Max[i] = 0;
		SensorData.u16Min[i] =4095;
	}
	SensorData.u16ADThreshold = 0;
	SensorData.u8EMIT_PWM = u16EmitterIntensity;		//this value shall be loaded from flash during initial
	SensorData.u8PosDetect = 0;
	SensorData.u8CalibState = 0;
	SensorData.u16PrevDelta = 0;
}

/**
  * @brief  Perform HeadUpData Initial
  * @param	None
  * @retval None
  */
void HeadUpData_Init(void){
	uint8_t i;
	for(i=0;i<8;i++){
		HeadUpData.u16Max[i] = 0;
		HeadUpData.u16Min[i] = 4095;
	}
	HeadUpData.u16ADTreshold = u16HeadUpThreshold;		//this value shall be loaded from flash during initial
	HeadUpData.u8State = HEAD_OPEN;
	HeadUpData.u8PrevState = HeadUpData.u8State;
}


/**
  * @brief  Perform Sensor Web and Threshold detect
  * @param	AD pointer of the GAP sensor AD sampling
  * @retval 0: normal, 1: GAP detected, 2: Error
  */
uint8_t GAP_Sense_Detect(uint32_t* u32ADPtr){
	uint16_t u16TempADC;
	uint16_t u16Length;
	uint8_t u8RetVal;
	uint8_t i,j;

	static uint16_t u16WebWidthStepCount=0;
	if(EngineData.u32StepCount == 0){
		for(i=0;i<8;i++){
			SensorData.u16Max[i] = 0;
			SensorData.u16Min[i] =4095;
		}
	}
	u8RetVal = 0;
	u16TempADC = (uint16_t)(*u32ADPtr);

	for(i=0;i<8;i++){
		if(u16TempADC > SensorData.u16Max[i]){
			for(j=i;j<7;j++){
				SensorData.u16Max[j+1] = SensorData.u16Max[j];
			}
			SensorData.u16Max[i]=u16TempADC;
			break;
		}
	}
	for(i=0;i<8;i++){
		if(u16TempADC < SensorData.u16Min[i]){
			for(j=i;j<7;j++){
				SensorData.u16Min[j+1] = SensorData.u16Min[j];
			}
			SensorData.u16Min[i] = u16TempADC;
			break;
		}
	}

	if(SensorData.u16ADThreshold){
		//Threshold has been calibrated
		if(SensorData.u8PosDetect == 0){
			if(u16TempADC > SensorData.u16ADThreshold+100){
				SensorData.u8PosDetect = 1;
				u16WebWidthStepCount = 0;
#ifdef DEBUG_IS_ON
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Rising Detected\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
#endif
			}
			u8RetVal = 0;
		}
		else{
			u16WebWidthStepCount++;
			if(u16WebWidthStepCount >8){
				if(u16TempADC < (SensorData.u16ADThreshold - 100)){
					SensorData.u8PosDetect = 0;
					//Renew Threshold
					SensorData.u16ADThreshold = SensorData.u16Max[0] - SensorData.u16Min[7];
					SensorData.u16PrevDelta = SensorData.u16Max[0] - SensorData.u16Min[7];
					SensorData.u16ADThreshold >>=1;
					SensorData.u16ADThreshold += SensorData.u16Min[7];
					u8RetVal = 1;
#ifdef DEBUG_IS_ON
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Gap Detected, Web:%d\r\n",u16WebWidthStepCount);
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
#endif
					u16WebWidthStepCount = 0;
				}
				if(u16WebWidthStepCount > 500){
					u8RetVal = 2;
#ifdef DEBUG_IS_ON
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Gap too wide\r\n");
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
#endif
					u16WebWidthStepCount = 0;
					SensorData.u8PosDetect = 0;
					SensorData.u16ADThreshold = 0;
					if(EngineData.u32StepCount == 0){
						for(i=0;i<8;i++){
							SensorData.u16Max[i] = 0;
							SensorData.u16Min[i] =4095;
						}
					}
				}
			}
		}
	}else{
		//Start threshold calibration
		if(SensorData.u8PosDetect == 0){
			if((u16TempADC == SensorData.u16Max[0] ) && (u16TempADC > (SensorData.u16Min[0] + (Z4000D_INTENSITY_LOW>>2) + 80))){
				//Rising detected
				SensorData.u8PosDetect = 1;
				u16WebWidthStepCount = 0;
#ifdef DEBUG_IS_ON
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Rising Detected\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
#endif
			}
			u8RetVal = 0;
		}
		else{
			u16WebWidthStepCount++;
			if(u16WebWidthStepCount > 8){
				if(u16TempADC < (SensorData.u16Min[0] + (Z4000D_INTENSITY_LOW>>2))){
					//Gap detected
					SensorData.u8PosDetect = 0;
					//Renew Threshold
					SensorData.u16ADThreshold = SensorData.u16Max[0] - SensorData.u16Min[0];
					SensorData.u16PrevDelta = SensorData.u16Max[0] - SensorData.u16Min[0];
					SensorData.u16ADThreshold >>= 1;
					SensorData.u16ADThreshold += SensorData.u16Min[0];
#ifdef DEBUG_IS_ON
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Gap Threshold %d Detected, Web:%d\r\n",SensorData.u16ADThreshold, u16WebWidthStepCount);
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
#endif
					u8RetVal = 1;
					u16WebWidthStepCount = 0;
				}
				if(u16WebWidthStepCount > 500){
#ifdef DEBUG_IS_ON
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Gap too wide");
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
#endif
					u16WebWidthStepCount = 0;
					SensorData.u8PosDetect = 0;
					for(i=0;i<8;i++){
						SensorData.u16Max[i] = 0;
						SensorData.u16Min[i] =4095;
					}
					u8RetVal = 2;
				}
			}
		}
	}
	return u8RetVal;
}

void Label_Calib(void){
	uint16_t u16HighLimit,u16LowLimit;
	uint16_t u16Length;
	u16HighLimit = Z4000D_INTENSITY_HIGH;
	u16LowLimit = Z4000D_INTENSITY_LOW;
	if((SensorData.u8CalibState & CALIB_ENABLE) && (SensorData.u8CalibState & CALIB_WITH_Z4000D)){
		HAL_GPIO_TogglePin(LED_RED_N_GPIO_Port,LED_RED_N_Pin);
		if(EngineData.u16State & ENGINE_READY){
			switch(SensorData.u8CalibState & 0x03){
			case 0:{
				if (HAL_TIM_PWM_Start(&htimSensor, TIM_CHANNEL_EMIT) != HAL_OK){
					/* PWM generation Error */
					Error_Handler();
				}
				__HAL_TIM_SET_COMPARE(&htimSensor, TIM_CHANNEL_EMIT, SensorData.u8EMIT_PWM);
#ifdef DEBUG_IS_ON
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"EMIT PWM = %d\r\n", SensorData.u8EMIT_PWM);
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
#endif
				PrintTask.u32ForFeedSteps = 100;
				//PrintTask.u8SensorType = USE_GAP_SENSOR;
				PrintTask.u8SetSpd = FEED_6_IPS;
				EngineData.u16State |= ENGINE_USEGAP;
				EngineData.u16State &= ~ENGINE_READY;
				EngineData.u16State |= ENGINE_START;
				SensorData.u8CalibState &= 0xF0;
				SensorData.u8CalibState |= 1;
			}break;
			case 1:{

				if(SensorData.u16PrevDelta >= u16HighLimit){
					//too high
					if(SensorData.u8EMIT_PWM>40){
						SensorData.u8EMIT_PWM -= 8;
						//__HAL_TIM_SET_COMPARE(&htimSensor, TIM_CHANNEL_EMIT, u16PWMValue);
					}
#ifdef DEBUG_IS_ON
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"delta = %d, too high\r\n", SensorData.u16PrevDelta);
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
#endif
					PrintTask.u32BackFeedSteps = 300;
					PrintTask.u8SetSpd = FEED_4_IPS;
					EngineData.u16State &= ~ENGINE_USEGAP;
					EngineData.u16State &= ~ENGINE_READY;
					EngineData.u16State |= ENGINE_START;
					SensorData.u8CalibState &= 0xF0;

				}else if(SensorData.u16PrevDelta <= u16LowLimit){
					//too low
					if(SensorData.u8EMIT_PWM<180){
						SensorData.u8EMIT_PWM += 15;
						//__HAL_TIM_SET_COMPARE(&htimSensor, TIM_CHANNEL_EMIT, u16PWMValue);
					}
#ifdef DEBUG_IS_ON
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"delta = %d, too low\r\n", SensorData.u16PrevDelta);
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
#endif
					PrintTask.u32BackFeedSteps = 300;
					PrintTask.u8SetSpd = FEED_4_IPS;
					EngineData.u16State &= ~ENGINE_USEGAP;
					EngineData.u16State &= ~ENGINE_READY;
					EngineData.u16State |= ENGINE_START;
					SensorData.u8CalibState &= 0xF0;
				}else{
					//good
					HAL_GPIO_WritePin(LED_RED_N_GPIO_Port, LED_RED_N_Pin, GPIO_PIN_SET);
					u16EmitterIntensity = SensorData.u8EMIT_PWM;		//this value shall be saved to flash in real version
#ifdef DEBUG_IS_ON
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Delta = %d\r\nCalib Done.\r\n", SensorData.u16PrevDelta);
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
#endif
					SensorData_Init();
				}
				SensorData.u16ADThreshold = 0;
			}break;
			}
		}
	}
}

/**
  * @brief  Perform Gap Sensor Intensity Calibration
  * @param	None
  * @retval None
  */
void Calib_Intensity(void){
	static uint16_t u16High,u16Low,u16Temp = 0;
	uint16_t u16HighLimit,u16LowLimit;
	uint16_t u16Length;
	if(SensorData.u8CalibState & CALIB_WITH_SWATCH){
		u16HighLimit = SWATCH_INTENSITY_HIGH;
		u16LowLimit = SWATCH_INTENSITY_LOW;
	}else{
		u16HighLimit = Z4000D_INTENSITY_HIGH;
		u16LowLimit = Z4000D_INTENSITY_LOW;
	}
	if((SensorData.u8CalibState & CALIB_ENABLE) && (EngineData.u16State & ENGINE_READY)){
		HAL_GPIO_TogglePin(LED_RED_N_GPIO_Port,LED_RED_N_Pin);
		switch (SensorData.u8CalibState & 0x03){
		case 0:{
			if (HAL_TIM_PWM_Start(&htimSensor, TIM_CHANNEL_EMIT) != HAL_OK)
			{
				/* PWM generation Error */
				Error_Handler();
			}
			__HAL_TIM_SET_COMPARE(&htimSensor, TIM_CHANNEL_EMIT, SensorData.u8EMIT_PWM);
#ifdef DEBUG_IS_ON
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"EMIT PWM = %d\r\n", SensorData.u8EMIT_PWM);
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
#endif
			SensorData.u8CalibState &= 0xF0;
			SensorData.u8CalibState |= 1;
		}break;
		case 1:{
			u16High = (uint16_t)(u32ADFiltered[ADQ_GAP]);
			PrintTask.u32ForFeedSteps = 240;
			PrintTask.u8SetSpd = FEED_4_IPS;
			EngineData.u16State &= ~ENGINE_READY;
			EngineData.u16State |= ENGINE_START;
#ifdef DEBUG_IS_ON
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"Liner AD = %d\r\n", u16High);
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
#endif
			SensorData.u8CalibState &= 0xF0;
			SensorData.u8CalibState |= 2;
		}break;
		case 2:{
			u16Low = (uint16_t)(u32ADFiltered[ADQ_GAP]);
			if(u16High>u16Low){
				u16Temp = u16High-u16Low;
				if(u16Temp > u16LowLimit){
					if(u16Temp > u16HighLimit){
						//too big
						if(SensorData.u8EMIT_PWM>40){
							SensorData.u8EMIT_PWM -= 8;
							//__HAL_TIM_SET_COMPARE(&htimSensor, TIM_CHANNEL_EMIT, u16PWMValue);
						}
						PrintTask.u32BackFeedSteps = 240;	//about 3 cm
						PrintTask.u8SetSpd = FEED_4_IPS;
						EngineData.u16State &= ~ENGINE_READY;
						EngineData.u16State |= ENGINE_START;
						SensorData.u8CalibState &= 0xF0;
#ifdef DEBUG_IS_ON
						if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
							sprintf((*CurrLogPtr).cdataptr,"Label AD= %d\r\nDelta = %d\r\nOver\r\n", u16Low, u16Temp);
							u16Length = strlen((*CurrLogPtr).cdataptr);
							(*CurrLogPtr).u16Length += u16Length;
							(*CurrLogPtr).cdataptr += u16Length;
						}
#endif
					}
					else{
						//Good
						HAL_GPIO_WritePin(LED_RED_N_GPIO_Port, LED_RED_N_Pin, GPIO_PIN_SET);
						u16EmitterIntensity = SensorData.u8EMIT_PWM;		//this value shall be saved to flash in real version
						if(SensorData.u8CalibState & CALIB_WITH_Z4000D){
							u8PrintTestMod = SENSOR_CURVE;
							Label_Loader(u8PrintTestMod, 1);
							EngineData.u16State &= ~ENGINE_READY;
							EngineData.u16State |= ENGINE_START;
						}else{
							PrintTask.u32ForFeedSteps = 900;	//feed out the swatch
							PrintTask.u8SetSpd = FEED_4_IPS;
							EngineData.u16State &= ~ENGINE_READY;
							EngineData.u16State |= ENGINE_START;
						}
						SensorData_Init();
#ifdef DEBUG_IS_ON
						if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
							sprintf((*CurrLogPtr).cdataptr,"Label AD = %d\r\nDelta = %d\r\nCalib Done.\r\n", u16Low, u16Temp);
							u16Length = strlen((*CurrLogPtr).cdataptr);
							(*CurrLogPtr).u16Length += u16Length;
							(*CurrLogPtr).cdataptr += u16Length;
						}
#endif
					}
				}
				else{
					//too small
					if(SensorData.u8EMIT_PWM< 180){
						SensorData.u8EMIT_PWM += 15;
						//__HAL_TIM_SET_COMPARE(&htimSensor, TIM_CHANNEL_EMIT, u16PWMValue);
					}
					PrintTask.u32BackFeedSteps =240;		//about 3 cm
					PrintTask.u8SetSpd = FEED_4_IPS;
					EngineData.u16State &= ~ENGINE_READY;
					EngineData.u16State |= ENGINE_START;
					SensorData.u8CalibState &= 0xF0;
#ifdef DEBUG_IS_ON
					if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
						sprintf((*CurrLogPtr).cdataptr,"Label AD = %d\r\nDelta = %d\r\nBelow\r\n", u16Low, u16Temp);
						u16Length = strlen((*CurrLogPtr).cdataptr);
						(*CurrLogPtr).u16Length += u16Length;
						(*CurrLogPtr).cdataptr += u16Length;
					}
#endif
				}
			}
			else{
				PrintTask.u32BackFeedSteps = 240; //about 3 cm
				PrintTask.u8SetSpd = FEED_4_IPS;
				EngineData.u16State &= ~ENGINE_READY;
				EngineData.u16State |= ENGINE_START;
#ifdef DEBUG_IS_ON
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"ERROR\r\n");
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
#endif
				SensorData.u8CalibState = 0;
			}
		}break;
		default:SensorData.u8CalibState = 0;break;
		}
	}
	return;
}

/**
  * @brief  Perform Gap Sensor switching on/off
  * @param	0:OFF, 1:ON
  * @retval None
  */
void GAP_Sensor_Switch(uint8_t u8OnOff){
	if(SensorData.u8CalibState & CALIB_ENABLE){
		//set the GAP sensor always on during Calibration
		return;
	}
	if(u8OnOff){
		__HAL_TIM_SET_COMPARE(&htimSensor, TIM_CHANNEL_EMIT, SensorData.u8EMIT_PWM);
	}
	else{
		__HAL_TIM_SET_COMPARE(&htimSensor, TIM_CHANNEL_EMIT, 0);
	}
	if (HAL_TIM_PWM_Start(&htimSensor, TIM_CHANNEL_EMIT) != HAL_OK){
		/* PWM generation Error */
		Error_Handler();
	}
}

/**
  * @brief  Perform Gap Curve Graphic Calculation
  * @param	ptr of ADValue
  * @retval None
  */
void Sensor_Curve_Calc(uint32_t* u32ADPtr, uint8_t* u8GraphicLine){
	uint8_t i, PrevWshift, CurrWshift, CurrDshift;
	uint16_t u16AdcVal;
	uint32_t u16Temp;
	static uint16_t u16PrevVal;
	memset((char *)u8GraphicLine,0x00, PH_BYTE_WIDTH);
	u16AdcVal = (uint16_t)(*u32ADPtr);
	u16AdcVal >>= 3;
	CurrDshift = (uint8_t)(u16AdcVal%8);
	CurrWshift = (uint8_t)(u16AdcVal/8);
	PrevWshift = (uint8_t)(u16PrevVal/8);
	if(CurrWshift>(PH_BYTE_WIDTH-2)){
		CurrWshift = (PH_BYTE_WIDTH-2);
	}
	if(PrevWshift>(PH_BYTE_WIDTH-2)){
		PrevWshift = (PH_BYTE_WIDTH-2);
	}
	//showing the ADC dot
	u8GraphicLine[CurrWshift] = 0x80;
	u8GraphicLine[CurrWshift] >>= CurrDshift;
	// connect the prev dot to current dot
	if(CurrWshift > (PrevWshift+1) ){
		for(i=(PrevWshift);i<(CurrWshift-1);i++){
			u8GraphicLine[i] = 0xff;
		}
	}else if((CurrWshift+1) < PrevWshift){
		for(i=(CurrWshift+1);i<(PrevWshift);i++){
			u8GraphicLine[i] = 0xff;
		}
	}
	// showing the threshold
	u16Temp = SensorData.u16ADThreshold>>3;
	CurrDshift = (uint8_t)(u16Temp%8);
	CurrWshift = (uint8_t)(u16Temp/8);
	u8GraphicLine[CurrWshift] = 0x80;
	u8GraphicLine[CurrWshift] >>= CurrDshift;
	u16PrevVal = u16AdcVal;
}

void Head_Open_Detection(uint32_t* u32AdcPtr){
	uint8_t i,j;
	uint16_t u16AdcVal;
	uint16_t u16Length;
	u16AdcVal = (uint16_t)(*u32AdcPtr);
	for(i=0;i<8;i++){
		if(u16AdcVal > HeadUpData.u16Max[i]){
			for(j=i;j<7;j++){
				HeadUpData.u16Max[j+1] = HeadUpData.u16Max[j];
			}
			HeadUpData.u16Max[i]=u16AdcVal;
			break;
		}
	}
	for(i=0;i<8;i++){
		if(u16AdcVal < HeadUpData.u16Min[i]){
			for(j=i;j<7;j++){
				HeadUpData.u16Min[j+1] = HeadUpData.u16Min[j];
			}
			HeadUpData.u16Min[i] = u16AdcVal;
			break;
		}
	}
	if(HeadUpData.u8State == HEAD_OPEN){
		u8LEDState = 7;
		if(HeadUpData.u16Max[7] > HeadUpData.u16ADTreshold){
			u16HeadUpThreshold = HeadUpData.u16Max[7] - 500;
#ifdef DEBUG_IS_ON
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"Head Close\r\nUpdated Threshold:%d\r\n",u16HeadUpThreshold );
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
#endif
			SensorData_Init();
			HeadUpData.u8State = HEAD_CLOSE;
			HeadUpData.u16ADTreshold = u16HeadUpThreshold;
			PrintTask_Init();
			EngineData_Init();
			u8LEDState = 0;
		}
	}else{
		if(HeadUpData.u16Min[7] < HeadUpData.u16ADTreshold){
			u16HeadUpThreshold = HeadUpData.u16Max[7] - 300;
#ifdef DEBUG_IS_ON
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"Head Open\r\nUpdated Threshold:%d\r\n",u16HeadUpThreshold );
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
#endif
			PrintEng_Init();
			HeadUpData.u8State = HEAD_OPEN;
			HeadUpData.u16ADTreshold = u16HeadUpThreshold;
		}else{

		}
	}
	HeadUpData.u8PrevState = HeadUpData.u8State;
}


