/*
 * thermal_print.c
 *
 *  Created on: Nov 3, 2017
 *      Author: KLam
 */
/* Includes -------------------------------------------------------------*/
#include "thermal_print.h"
#include "table.h"
#include "misc.h"
#include "comm_handle.h"
#include "mem_handle.h"
#include "sensor_handle.h"
#include "usbd_cdc_if.h"
#include "test_mod.h"
#include <stdint.h>
#include <string.h>
/* Variables Definitions ------------------------------------------------*/
PRINT_TASK_TYPE PrintTask;
ENGINE_DATA_TYPE EngineData;

uint32_t u32BlankLine[26];
uint32_t u32DynamicGraphic[26];

uint8_t u8MTRCurrentLimitPWM=140;

uint16_t u16ContPrintedPages=0;

/* Function Definitions -------------------------------------------------*/
uint8_t Strobe_Time_Calc(void);
void Strobe_Data_Calc(uint32_t* u32NewLinePtr, uint16_t u16LineWidth);
uint8_t Label_Handle(void);
void Load_Next_Label(void);

/**
  * @brief  Latch ISR, call in Latch timer interrupt
  * @param	None
  * @retval None
  */
void Strobe_ISR(void){
	if(HAL_GPIO_ReadPin(PH_STR_1_GPIO_Port, PH_STR_1_Pin) == GPIO_PIN_RESET){
		HAL_GPIO_WritePin(PH_STR_1_GPIO_Port, PH_STR_1_Pin, GPIO_PIN_SET);	//Pull up strobe
#ifdef STROBE_COUNT_IS_3
		__HAL_TIM_SET_AUTORELOAD(&htimStrobe, EngineData.u16StrobeTime[EngineData.u8StrobeSeq+1]);
#else
		__HAL_TIM_SET_AUTORELOAD(&htimStrobe, EngineData.u16StrobeTime[EngineData.u8StrobeSeq]);
#endif
		__HAL_TIM_SET_COUNTER(&htimStrobe, 0);
		HAL_TIM_Base_Start_IT(&htimStrobe);
	}else{
		HAL_GPIO_WritePin(PH_STR_1_GPIO_Port, PH_STR_1_Pin, GPIO_PIN_RESET);
		HAL_TIM_Base_Stop_IT(&htimStrobe);
	}
}
/**
  * @brief  Latch ISR, call in Strobe timer interrupt
  * @param	None
  * @retval None
  */
void Latch_ISR(void){
	if(EngineData.u16State & (ENGINE_MTRON|ENGINE_PRINTON)){
		/* Latch signal lock in last Data */
		HAL_GPIO_WritePin(PH_LATCH_GPIO_Port, PH_LATCH_Pin, GPIO_PIN_SET);//Pull down Latch, inverted
#ifdef STROBE_COUNT_IS_3
		__HAL_TIM_SET_AUTORELOAD(&htimLatch, EngineData.u16LatchTime[EngineData.u8StrobeSeq+1]);
#else
		__HAL_TIM_SET_AUTORELOAD(&htimLatch, EngineData.u16LatchTime[EngineData.u8StrobeSeq]);
#endif
		__HAL_TIM_SET_COUNTER(&htimLatch, 0);
		HAL_GPIO_WritePin(PH_LATCH_GPIO_Port, PH_LATCH_Pin, GPIO_PIN_RESET); //Pull up Latch, inverted
		HAL_TIM_Base_Start_IT(&htimLatch);
		/* Trigger the strobe for print */
		Strobe_ISR();

		EngineData.u8StrobeSeq++;
#ifdef STROBE_COUNT_IS_3
		if(EngineData.u8StrobeSeq >2){
#else
		if(EngineData.u8StrobeSeq >3){
			//This is Strobe 4, calculation next 4 strobe and data
#endif
			EngineData.u8StrobeSeq = 0;
			HAL_TIM_Base_Stop_IT(&htimLatch);		//it is not necessary for last strobe
			//calc data for next
			Strobe_Time_Calc();
			if(EngineData.u32PrintLineCount>1){
				Strobe_Data_Calc((uint32_t*)EngineData.u8DataPtr, EngineData.u16PrintLineWidth);
				if(EngineData.u16State & ENGINE_IMAGEPRINT){
					EngineData.u8DataPtr += (EngineData.u16PrintLineWidth/8);
				}
			}
			else{
				//final two lines shall load in blank line as the the last line has been loaded in last 3 line.
				Strobe_Data_Calc(u32BlankLine, EngineData.u16PrintLineWidth);
			}
		}
		if(DebugSwitch & PH_SPI_ON){
			/* Send data to SPI port */
#ifdef STROBE_COUNT_IS_3
			HAL_SPI_Transmit_DMA(&hspiPH, (uint8_t *)&EngineData.u32StrobeData[EngineData.u8StrobeSeq+1][0], 104);//(EngineData.u16PrintLineWidth/8));
#else
			HAL_SPI_Transmit_DMA(&hspiPH, (uint8_t *)&EngineData.u32StrobeData[EngineData.u8StrobeSeq][0], 104);//(EngineData.u16PrintLineWidth/8));
#endif
		}
	}else{
		//there shall be error
		Stop_MTR();
		PrintTask_Init();
	}
}

/**
  * @brief  Stepper ISR, call in stepper timer interrupt
  * @param	None
  * @retval None
  */
void Stepper_FullHalf_ISR(void){
	static uint8_t u8AccInterval = 0;
	if(EngineData.u16State & ENGINE_MTRON){
		switch(EngineData.u8StepPhase){
		case 0:{
			if(EngineData.u32StepCount > 0){
				EngineData.u32StepCount --;
				HAL_GPIO_WritePin(MTR_STEP_GPIO_Port, MTR_STEP_Pin, GPIO_PIN_SET);		//Rise of step
			}
			else{
				Stop_MTR();
			}
			/* Thermal Print Handle */
			if(EngineData.u16State & ENGINE_PRINTON){
				if(EngineData.u32PrintLineCount){
					EngineData.u32PrintLineCount--;
					Latch_ISR();
				}
				else{
					EngineData.u16State &= ~(ENGINE_PRINTON);
				}
			}
			Label_Handle();
			//Image generator
			if(EngineData.u16State & ENGINE_SENSECURVE){
				Sensor_Curve_Calc(&(u32ADFiltered[ADQ_GAP]), (uint8_t*)(u32DynamicGraphic));
			}
			if(EngineData.u16State & ENGINE_PIXELGRAPHIC){
				Pixel_Graphic_Calc();
			}
			if(EngineData.u16State & ENGINE_FENCEGRAPHIC){
				Fence_Graphic_Calc();
			}
			EngineData.u8StepPhase = 1;
		}break;
		case 1:{
			HAL_GPIO_WritePin(MTR_STEP_GPIO_Port, MTR_STEP_Pin, GPIO_PIN_RESET);	//Fall of step
			HAL_GPIO_WritePin(LED_RED_N_GPIO_Port, LED_RED_N_Pin, GPIO_PIN_SET);		//for debug

			EngineData.u8StepPhase = 2;
		}break;
		case 2:{
#ifdef USE_FULL_STEP
			HAL_GPIO_WritePin(MTR_STEP_GPIO_Port, MTR_STEP_Pin, GPIO_PIN_RESET);
#else
			HAL_GPIO_WritePin(MTR_STEP_GPIO_Port, MTR_STEP_Pin, GPIO_PIN_SET);
#endif
			EngineData.u8StepPhase = 3;
		}break;
		case 3:{
			HAL_GPIO_WritePin(MTR_STEP_GPIO_Port, MTR_STEP_Pin, GPIO_PIN_RESET);
			//Acceleration calculation
			if(EngineData.u16CurrStepperPeriod != EngineData.u16PrevStepperPeriod){
				__HAL_TIM_SET_AUTORELOAD(&htimStepper,EngineData.u16CurrStepperPeriod);
				EngineData.u16PrevStepperPeriod = EngineData.u16CurrStepperPeriod;
			}else{
				if(++u8AccInterval>1){		//step length is 2
					u8AccInterval = 0;
					if(EngineData.u32StepCount > (PrintTask.u8SetSpd*2)){	//decrease step length is 2
						//ramp up
						if(EngineData.u8Spd < PrintTask.u8SetSpd){
							EngineData.u8Spd++;
						}
						else{
							//stable decrease PWM
							if(EngineData.u8RefcurrPWM > u8MTRCurrentLimitPWM){//u8MTRCurrLimitTable[EngineData.u8Spd]){
								EngineData.u8RefcurrPWM--;
								__HAL_TIM_SET_COMPARE(&htimMTRCURREF, TIM_CHANNEL_MTRCURREF, EngineData.u8RefcurrPWM);
								if (HAL_TIM_PWM_Start(&htimMTRCURREF, TIM_CHANNEL_MTRCURREF) != HAL_OK)
								{
									/* PWM generation Error */
									Error_Handler();
								}
							}
						}

					}
					else{
						//ramp down
						if(EngineData.u8Spd){
							EngineData.u8Spd--;
						}
					}
					EngineData.u16CurrStepperPeriod = u16StepperAccTableHalf[EngineData.u8Spd];
				}
			}
			EngineData.u8StepPhase = 0;
		}break;
		default:{
			Stop_MTR();
		}break;
		}
	}
	else{
		//should be error state
		//HAL_TIM_Base_Stop_IT(&htimStepper);
		Stop_MTR();
	}
}

/**
  * @brief  Label_Handle
  * @param	None
  * @retval 0: ok ,>0 error code
  */
uint8_t Label_Handle(void){
	uint8_t u8RetVal;
	uint16_t u16Length;
	static uint32_t u32BlankLineCount;
	u8RetVal = 0;
	if(EngineData.u16State & ENGINE_USEGAP){
		switch(GAP_Sense_Detect(&u32ADFiltered[ADQ_GAP])){
			case 0:{
				//no gap found in current line
				if(EngineData.u16State & ENGINE_GAPFOUND){
					//GAP is found before
					if(u32BlankLineCount){
						u32BlankLineCount--;
					}
					if(u32BlankLineCount == 0){
						if(EngineData.u16State & ENGINE_OVERHEAT){
							EngineData.u16State &= ~ENGINE_GAPFOUND;
						}else{
							EngineData.u8PHTempC = PH_Temp_Calc(&u32ADFiltered[ADQ_PHTEMP]);
							//Debug message
#ifdef DEBUG_IS_ON
							if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
								sprintf((*CurrLogPtr).cdataptr,"PH Temp = %d\r\n%d Label left\r\n",(int)EngineData.u8PHTempC, (int)PrintTask.u32LabelCounter);
								u16Length = strlen((*CurrLogPtr).cdataptr);
								(*CurrLogPtr).u16Length += u16Length;
								(*CurrLogPtr).cdataptr += u16Length;
							}
#endif
							if(1){
								u16ContPrintedPages ++;
#ifdef DEBUG_IS_ON
								if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
									sprintf((*CurrLogPtr).cdataptr,"%d Labels printed continuously\r\n",(int)u16ContPrintedPages);
									u16Length = strlen((*CurrLogPtr).cdataptr);
									(*CurrLogPtr).u16Length += u16Length;
									(*CurrLogPtr).cdataptr += u16Length;
								}
#endif

							}
							if((EngineData.u8PHTempC >= MAX_PRINT_TEMP) && (PrintTask.u32LabelCounter)){
								EngineData.u16State |= ENGINE_OVERHEAT;
								u32BlankLineCount = (DISTANCE_GAPTOTEAR-DISTANCE_GAPTOHEAD);
								EngineData.u32StepCount = u32BlankLineCount;

							}else{
								EngineData.u16State &= ~ENGINE_GAPFOUND;
								//sprintf((char*)u8TxBuf, "Label=%d\r\n", PrintTask.u32LabelCounter);
								Load_Next_Label();
							}
						}
					}
				}else{
					//GAP is not found
					if(EngineData.u32StepCount < DISTANCE_GAPTOHEAD){
						//GAP is not found, Label will be end, need add steps to find next GAP
						EngineData.u32StepCount += DISTANCE_GAPTOHEAD;
					}
					if(EngineData.u32PrintLineCount > 0){
						u32BlankLineCount = 0;
					}
					else{
						u32BlankLineCount++;
						if(u32BlankLineCount > MAX_LABEL_LENGTH){
							//no GAP found for long feeding
#ifdef DEBUG_IS_ON
							if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
								sprintf((*CurrLogPtr).cdataptr,"Feed Jam or Continuous Media\r\n");
								u16Length = strlen((*CurrLogPtr).cdataptr);
								(*CurrLogPtr).u16Length += u16Length;
								(*CurrLogPtr).cdataptr += u16Length;
							}
#endif

							u32BlankLineCount = 0;
							EngineData_Init();
							PrintTask_Init();
							EngineData.u16State |= ENGINE_ERROR;
							u8LEDState = 7;
							u8RetVal = 1;
						}

					}
				}
			}break;
			case 1:{
				//GAP is found
				if(EngineData.u32PrintLineCount < DISTANCE_GAPTOHEAD){
					//Label will be finished before the GAP
					EngineData.u16State |= ENGINE_GAPFOUND;
					if(PrintTask.u32LabelCounter){
						//There is task in Q
						u32BlankLineCount = DISTANCE_GAPTOHEAD;
						EngineData.u32StepCount = u32BlankLineCount+100;
					}else{
						//There shall be and end on tear bar
						u32BlankLineCount = DISTANCE_GAPTOTEAR;
						EngineData.u32StepCount = u32BlankLineCount;
					}
				}else{
					//Label cannot be finished before the GAP
					if(EngineData.u16State & (ENGINE_SENSECURVE|ENGINE_PIXELGRAPHIC|ENGINE_FENCEGRAPHIC)){ //Limit to 1 label for curve detection
						EngineData.u16State |= ENGINE_GAPFOUND;
						EngineData.u32PrintLineCount = (DISTANCE_GAPTOHEAD-20);
						u32BlankLineCount = DISTANCE_GAPTOTEAR;
						EngineData.u32StepCount = u32BlankLineCount;
					}
				}
			}break;
			default:{
				u32BlankLineCount = 0;
				EngineData_Init();
				PrintTask_Init();
				EngineData.u16State |= ENGINE_ERROR;
				u8LEDState = 7;
				u8RetVal = 2;//error
			}break;
		}//end of switch gap status
	}//end of if(PrintTask.u8SensorType & USE_GAP_SENSOR)
	return u8RetVal;
}


/**
  * @brief  Perform Gap Detection, call during feeding only
  * @param	None
  * @retval 0: normal, 1: gap detect, >1, error
  */

/*
 * Function:	PrintEng_Init
 * Description:	Initial the buffer and I/O for print
 * Input:		N/A
 * Return:		N/A
 */
void PrintEng_Init(void){
	//Motor Driver Initial
	HAL_GPIO_WritePin(MTR_RST_N_GPIO_Port, MTR_RST_N_Pin, GPIO_PIN_RESET);		//RESET = 0
	//Delay 50ms
	HAL_Delay(50);
	//Set values
	HAL_GPIO_WritePin(MTR_SLEEP_GPIO_Port, MTR_SLEEP_Pin, GPIO_PIN_SET);		//SLEEP = 1
	HAL_GPIO_WritePin(MTR_TRQ1_GPIO_Port, MTR_TRQ1_Pin, GPIO_PIN_RESET);		//TRQ1 = 0,
#ifdef USE_FULL_STEP
	HAL_GPIO_WritePin(MTR_USM0_GPIO_Port, MTR_USM0_Pin, GPIO_PIN_RESET);		//Full Step USM0 = 0 USM1 = 0
#else
	HAL_GPIO_WritePin(MTR_USM0_GPIO_Port, MTR_USM0_Pin, GPIO_PIN_SET);			//Full Step USM0 = 1 USM1 = 0
#endif
	HAL_GPIO_WritePin(MTR_USM1_GPIO_Port, MTR_USM1_Pin, GPIO_PIN_RESET);		//USM0 = 1 USM1 = 0
	HAL_GPIO_WritePin(MTR_DIR_GPIO_Port, MTR_DIR_Pin, MTR_DIR_PIN_VALUE);			//DIR = 1;
	HAL_GPIO_WritePin(MTR_STEP_GPIO_Port, MTR_STEP_Pin, GPIO_PIN_RESET);		//STEP = 0
	HAL_GPIO_WritePin(MTR_EN_GPIO_Port, MTR_EN_Pin, GPIO_PIN_SET);				//ENABLE = 1, disable
	HAL_GPIO_WritePin(MTR_RST_N_GPIO_Port, MTR_RST_N_Pin, GPIO_PIN_SET);		//RESET = 1

	/* Initial PH GPIO */
	HAL_GPIO_WritePin(PH_STR_1_GPIO_Port, PH_STR_1_Pin, GPIO_PIN_RESET);		//STR = 0
	HAL_GPIO_WritePin(PH_LATCH_GPIO_Port, PH_LATCH_Pin, GPIO_PIN_RESET);			//LATCH = 1 Inverted
	HAL_GPIO_WritePin(PH_POWER_ON_GPIO_Port, PH_POWER_ON_Pin, GPIO_PIN_RESET);	//PH_POWER_ON = 0;

	/* Initial MTR CURREF */
	if (HAL_TIM_PWM_Start(&htimMTRCURREF, TIM_CHANNEL_MTRCURREF) != HAL_OK)
	{
		/* PWM generation Error */
		Error_Handler();
	}
	EngineData.u8RefcurrPWM = 199;
	__HAL_TIM_SET_COMPARE(&htimMTRCURREF, TIM_CHANNEL_MTRCURREF, EngineData.u8RefcurrPWM);		//set current limit to max

	/* Initial Sensors */
	HeadUpData_Init();
	SensorData_Init();

	//Data Initial
	EngineData_Init();
	PrintTask_Init();
	SDRAM_Buff_Init();
	LogBuf_Init();
	if (HAL_TIM_PWM_Start(&htimSensor, TIM_CHANNEL_EMIT) != HAL_OK){
		/* PWM generation Error */
		Error_Handler();
	}
	__HAL_TIM_SET_COMPARE(&htimSensor, TIM_CHANNEL_EMIT, SensorData.u8EMIT_PWM);
	//Stop all timer
	HAL_TIM_Base_Stop_IT(&htimStepper);
	HAL_TIM_Base_Stop_IT(&htimStrobe);
	HAL_TIM_Base_Stop_IT(&htimLatch);
	GAP_Sensor_Switch(0);
}

/**
  * @brief  Perform PH NTC Temperature Calculation
  * @param	u32ADCPtr: Pointer to the AD value
  * @retval C temperature 0 to 101
  */
uint8_t PH_Temp_Calc(uint32_t* u32ADCPtr){
	uint8_t i;
	uint16_t u16TempADC;
	u16TempADC = (uint16_t)*u32ADCPtr;
	for(i=0;i<26;i++){
		if(u16TempADC > u16PHTempTblM[i])break;
	}
	if(i>0){
		if(i>24){
			//NTC resistance too low, too cold
			return 100;
		}
		else{
			return (i*4-(u16TempADC-u16PHTempTblM[i])/u16PHTempTblL[(i-1)]);
		}
	}
	else{
		//NTC resistance too high, too cold
		return 0;
	}
}


/**
  * @brief  Perform Strobe Time calculation
  * @param	None
  * @retval 0 is correct, 1 ~ x error code
  */
uint8_t Strobe_Time_Calc(void){
	uint8_t i;
	int16_t s16TempComp;
	int16_t s16TimeComp;
	uint16_t u16TotalTime;
	uint16_t u16MaxStrobeTime;
	uint16_t u16Length;
	s16TempComp = (int16_t)EngineData.u8PHTempC - 25;
	u16MaxStrobeTime = u16StepperAccTableHalf[EngineData.u8Spd]*2;
	//Fixed time for STRB 0 and 1
#ifdef STROBE_COUNT_IS_3
	EngineData.u16StrobeTime[0] = STROBE_0_TIME + EngineData.u8Darkness*u16StbDarkness[EngineData.u8Spd];
	EngineData.u16StrobeTime[1] = STROBE_1_TIME + EngineData.u8Darkness*u16StbDarkness[EngineData.u8Spd];;
#else
	EngineData.u16StrobeTime[0] = STROBE_0_TIME + EngineData.u8Darkness*u16StbDarkness[EngineData.u8Spd];
	EngineData.u16StrobeTime[1] = STROBE_1_TIME;
#endif
	//Temperature compensate STRB 2 and 3
	s16TimeComp = s16TempComp * u16StbTempComp[EngineData.u8Spd];
	s16TimeComp /= 2;
	//Debug message
#ifdef STROBE_COUNT_IS_3
	EngineData.u16StrobeTime[2] = 60+u16StrobeTable[EngineData.u8Spd]+2*EngineData.u8Darkness*u16StbDarkness[EngineData.u8Spd]-s16TimeComp;
#else
	EngineData.u16StrobeTime[2] = u16StrobeTable[EngineData.u8Spd]+2*EngineData.u8Darkness*u16StbDarkness[EngineData.u8Spd]-s16TimeComp;
#endif
	if(EngineData.u16StrobeTime[2] > u16MaxStrobeTime){
		//Strobe time is longer than half of step time
#ifdef DEBUG_IS_ON
		if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
			sprintf((*CurrLogPtr).cdataptr,"strobe time %d > %d\r\n", EngineData.u16StrobeTime[2],u16MaxStrobeTime);
			u16Length = strlen((*CurrLogPtr).cdataptr);
			(*CurrLogPtr).u16Length += u16Length;
			(*CurrLogPtr).cdataptr += u16Length;
		}
#endif
		EngineData.u16StrobeTime[2] = u16MaxStrobeTime;
	}
	EngineData.u16StrobeTime[3] = EngineData.u16StrobeTime[2]>>1;
	//Comparing with SPI port time, if it is < SPI time, using minimum latch time
	u16TotalTime = 0;
	for(i=0;i<4;i++){
		if((EngineData.u16StrobeTime[i]+3) > MIN_LATCH_TIME ){
			EngineData.u16LatchTime[i] = EngineData.u16StrobeTime[i]+3;
		}
		else{
			EngineData.u16LatchTime[i] = MIN_LATCH_TIME;
		}
		u16TotalTime +=EngineData.u16LatchTime[i];
	}
#ifdef STROBE_COUNT_IS_3
	u16TotalTime -= EngineData.u16LatchTime[0];
#endif
	//Check if over step time
	if((u16TotalTime+10)> (u16StepperAccTableHalf[EngineData.u8Spd]<<2)){
		//over step time limitation
		//Debug message
#ifdef DEBUG_IS_ON
		if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
			sprintf((*CurrLogPtr).cdataptr,"Strb total:%d, Step Time:%d\r\n",(int)u16TotalTime,(int)(u16StepperAccTableHalf[EngineData.u8Spd]<<2));
			u16Length = strlen((*CurrLogPtr).cdataptr);
			(*CurrLogPtr).u16Length += u16Length;
			(*CurrLogPtr).cdataptr += u16Length;
		}
#endif
		return 1;
	}

	return 0;
}

/**
  * @brief  Perform EngineData Initial
  * @param	None
  * @retval None
  */
void EngineData_Init(void){
	memset(u32BlankLine,0,sizeof(u32BlankLine));
	memset(EngineData.u32LineBuf,0,sizeof(EngineData.u32LineBuf));
	memset(EngineData.u32StrobeData,0,sizeof(EngineData.u32StrobeData));
	memset(EngineData.u16LatchTime,0,sizeof(EngineData.u16LatchTime));
	memset(EngineData.u16StrobeTime,0,sizeof(EngineData.u16StrobeTime));
	EngineData.u16State = ENGINE_READY;
	EngineData.u8PHTempC = PH_Temp_Calc(&u32ADFiltered[ADQ_PHTEMP]);
	EngineData.u8LineBufPtr = 0;
	EngineData.u8Spd = 0;
	EngineData.u8StepPhase = 0;
	EngineData.u8StrobeSeq = 0;
	EngineData.u32StepCount = 0;
	EngineData.u32PrintLineCount = 0;
	EngineData.u8RefcurrPWM = 199;
	EngineData.u8Darkness = 14;
	return;
}

/**
  * @brief  Perform Print Task Initial
  * @param	None
  * @retval None
  */
void PrintTask_Init(void){
	PrintTask.u32BackFeedSteps = 0;
	PrintTask.u32ForFeedSteps = 0;
	PrintTask.u32LabelCounter = 0;
	PrintTask.u32LabelLength = 0;
	PrintTask.u8SensorType = 0;
	//PrintTask.u8SetDensity = 0;
	PrintTask.u8SetSpd = FEED_6_IPS;
	PrintTask.u8SetDensity = 14;
	PrintTask.u8DataHeadPtr =(uint8_t*)(u32BlankLine);
	PrintTask.u16Status = TASK_EMPTY;
	return;
}

/**
  * @brief  Perform Print Task Start
  * @param	None
  * @retval 0 or Error Code
  */
uint8_t PrintTask_Handler(void){
	//uint8_t i;
	static uint8_t u8PrevTempC;
	uint16_t u16Length;
	//PH Temperature check
	if(EngineData.u16State & ENGINE_ERROR){
		u8LEDState = 7;
		Stop_MTR();
		return 5;
	}
	if(EngineData.u16State & ENGINE_OVERHEAT){
		u8LEDState = 7;
		EngineData.u8PHTempC = PH_Temp_Calc(&u32ADFiltered[ADQ_PHTEMP]);
		if(EngineData.u8PHTempC > (MAX_PRINT_TEMP-5)){
			if(EngineData.u8PHTempC != u8PrevTempC){
#ifdef DEBUG_IS_ON
				if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
					sprintf((*CurrLogPtr).cdataptr,"Print Head Over Heat: %dC\r\n",(int)EngineData.u8PHTempC);
					u16Length = strlen((*CurrLogPtr).cdataptr);
					(*CurrLogPtr).u16Length += u16Length;
					(*CurrLogPtr).cdataptr += u16Length;
				}
#endif
				u8PrevTempC = EngineData.u8PHTempC;
			}

		}else{
			EngineData.u16State &= ~ENGINE_OVERHEAT;
			if(PrintTask.u32LabelCounter){
				PrintTask.u32BackFeedSteps = DISTANCE_HEADTOTEAR;
				PrintTask.u8SetSpd = FEED_3_IPS;
				//PrintTask.u8SensorType = 0;
				EngineData.u16State &= ~ENGINE_USEGAP;
				u8LEDState = 0;
			}
		}
		return 4;		// jump out if it is over heat
	}

	if(EngineData.u16State & ENGINE_READY){
		EngineData.u8PHTempC = PH_Temp_Calc(&u32ADFiltered[ADQ_PHTEMP]);
		if(EngineData.u8PHTempC > MAX_PRINT_TEMP){
			EngineData.u16State |= ENGINE_OVERHEAT;
			return 4;
		}
	}

	//Task handling
	if(EngineData.u16State & ENGINE_START){
		//Check Print Task
		if(PrintTask.u32BackFeedSteps > 0){
			if(EngineData.u16State & ENGINE_MTRON){
				//motor is running
				return 1;
			}else{
				//PrintTask.u8SetSpd = FEED_3_IPS;
				if(PrintTask.u32BackFeedSteps < (PrintTask.u8SetSpd<<2)){
					//Steps length not long enough for ramp up and down
					PrintTask.u8SetSpd = PrintTask.u32BackFeedSteps>>2;
				}
				EngineData.u32StepCount = PrintTask.u32BackFeedSteps;
				PrintTask.u32BackFeedSteps = 0;
				//PrintTask.u8SensorType = 0;
				EngineData.u16State &= ~ENGINE_USEGAP;
				EngineData.u16State &= ~ENGINE_START;
				Start_MTR(!MTR_DIR_PIN_VALUE);
				return 0;
			}
		}
		else if(PrintTask.u32ForFeedSteps > 0){
			if(EngineData.u16State & ENGINE_MTRON){
				//motor is running
				return 2;
			}
			else{
				if(PrintTask.u32ForFeedSteps < (PrintTask.u8SetSpd<<2)){
					//Steps length not long enough for ramp up and down
					PrintTask.u8SetSpd = PrintTask.u32BackFeedSteps>>2;
				}
				EngineData.u32StepCount = PrintTask.u32ForFeedSteps;
				PrintTask.u32ForFeedSteps = 0;
				//PrintTask.u8SensorType = 0;
				EngineData.u16State &= ~ENGINE_START;
				Start_MTR(MTR_DIR_PIN_VALUE);

				return 0;
			}
		}
		else if(PrintTask.u32LabelCounter > 0){
			if(EngineData.u16State & ENGINE_MTRON){
				//motor is running

				return 3;
			}
			else{
				Load_Next_Label();
				EngineData.u16State &= ~ENGINE_START;
				Start_MTR(MTR_DIR_PIN_VALUE);
				return 0;
			}
		}
		else{
			EngineData.u16State &= ~ENGINE_START;
			EngineData.u16State |= ENGINE_READY;
			return 0;
		}
	}else if(EngineData.u16State & ENGINE_DONE){
		if(EngineData.u16State & ENGINE_MTRON){
			//motor is running
			return 3;
		}else{
			EngineData.u16State &= ~ENGINE_DONE;
			EngineData.u16State |= ENGINE_START;
			return 0;
		}
	}else{
		return 0;
	}
}


/**
  * @brief  Perform Motor Start up
  * @param	DIR Pin State
  * @retval None
  */
void Start_MTR(GPIO_PinState PinState){
	uint16_t u16Length;
	if(HeadUpData.u8State == HEAD_OPEN){
#ifdef DEBUG_IS_ON
		if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
			sprintf((*CurrLogPtr).cdataptr,"Head Open\r\n");
			u16Length = strlen((*CurrLogPtr).cdataptr);
			(*CurrLogPtr).u16Length += u16Length;
			(*CurrLogPtr).cdataptr += u16Length;
		}
#endif
		return;
	}
	//SensorData_Init();
	GAP_Sensor_Switch(1);
	HAL_Delay(200);		//wait for GAP sensor PWM working
	HAL_GPIO_WritePin(PH_POWER_ON_GPIO_Port, PH_POWER_ON_Pin, GPIO_PIN_SET);	//Power up Engine
	HAL_GPIO_WritePin(MTR_RST_N_GPIO_Port, MTR_RST_N_Pin, GPIO_PIN_SET);
#ifdef USE_FULL_STEP
	HAL_GPIO_WritePin(MTR_USM0_GPIO_Port, MTR_USM0_Pin, GPIO_PIN_RESET);
#else
	HAL_GPIO_WritePin(MTR_USM0_GPIO_Port, MTR_USM0_Pin, GPIO_PIN_SET);			//USM0 = 1 USM1 = 0
#endif
	HAL_GPIO_WritePin(MTR_USM1_GPIO_Port, MTR_USM1_Pin, GPIO_PIN_RESET);		//USM0 = 1 USM1 = 0
	HAL_GPIO_WritePin(MTR_TRQ1_GPIO_Port, MTR_TRQ1_Pin, GPIO_PIN_RESET);		//TRQ1 = 0 high limitation
	HAL_GPIO_WritePin(MTR_STEP_GPIO_Port, MTR_STEP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MTR_DIR_GPIO_Port, MTR_DIR_Pin, PinState);
	EngineData.u8Spd = 0;
	EngineData.u8StepPhase = 0;
	EngineData.u8StrobeSeq = 0;
	EngineData.u8RefcurrPWM = 199;
	__HAL_TIM_SET_COMPARE(&htimMTRCURREF, TIM_CHANNEL_MTRCURREF, EngineData.u8RefcurrPWM);
	//u8MTRCurrentLimitPWM = 140;
	EngineData.u16CurrStepperPeriod = u16StepperAccTableHalf[0];
	__HAL_TIM_SET_AUTORELOAD(&htimStepper,EngineData.u16CurrStepperPeriod);
	EngineData.u16PrevStepperPeriod = EngineData.u16CurrStepperPeriod;
	EngineData.u16State |= ENGINE_MTRON;
	__HAL_TIM_SET_COUNTER(&htimStepper,0);
	HAL_GPIO_WritePin(MTR_EN_GPIO_Port, MTR_EN_Pin, GPIO_PIN_RESET);			//Enable MTR Driver
	HAL_TIM_Base_Start_IT(&htimStepper);
}


/**
  * @brief  Perform Motor Start up
  * @param	DIR Pin State
  * @retval None
  */
void Stop_MTR(void){
	GAP_Sensor_Switch(0);
	HAL_TIM_Base_Stop_IT(&htimStepper);
	HAL_TIM_Base_Stop_IT(&htimStrobe);
	HAL_TIM_Base_Stop_IT(&htimLatch);
	HAL_SPI_DMAStop(&hspiPH);
	HAL_GPIO_WritePin(MTR_EN_GPIO_Port, MTR_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PH_POWER_ON_GPIO_Port, PH_POWER_ON_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PH_STR_1_GPIO_Port, PH_STR_1_Pin, GPIO_PIN_RESET);		//STR = 0
	HAL_GPIO_WritePin(PH_LATCH_GPIO_Port, PH_LATCH_Pin, GPIO_PIN_RESET); 			//Pull up Latch
	//EngineData_Init();
	EngineData.u16State &= ~(ENGINE_MTRON);
	EngineData.u16State |= ENGINE_DONE;
	u16ContPrintedPages = 0;
}

/**
  * @brief  Perform Next line Strobe Data Calculation,
  * @param	Ptr of new line data, will be loaded in future 2nd line
  * @retval None
  */
#ifdef STROBE_COUNT_IS_3
void Strobe_Data_Calc(uint32_t* u32NewLinePtr, uint16_t u16LineWidth){
	uint8_t i;
	uint8_t h1,h2,h3,h4,h5,f1,f2;
	uint32_t u32temp5,u32temp4,u32temp3,u32temp2,u32temp1;	//history 1 to 5
	uint32_t u32tempL1, u32tempL2;	//history before 1 blank line
	uint32_t u32tempH1, u32tempH2;	//history before 2 blank line
	uint32_t u32tempF1, u32tempF2;	//future lines
	uint16_t u16WordCount;
	u16WordCount = u16LineWidth/32;

	h1 = (EngineData.u8LineBufPtr+7) & 0x07;
	h2 = (EngineData.u8LineBufPtr+6) & 0x07;
	h3 = (EngineData.u8LineBufPtr+5) & 0x07;
	h4 = (EngineData.u8LineBufPtr+4) & 0x07;
	h5 = (EngineData.u8LineBufPtr+3) & 0x07;
	f1 = (EngineData.u8LineBufPtr+1) & 0x07;
	f2 = (EngineData.u8LineBufPtr+2) & 0x07;
	for(i=0;i<u16WordCount;i++){
		EngineData.u32LineBuf[f2][i] = u32NewLinePtr[i];	//load data to recent 8 lines
		EngineData.u32StrobeData[3][i] = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];	//load raw data to main strobe 3
		EngineData.u32StrobeData[2][i] = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];	//load raw data to main strobe 2
		EngineData.u32StrobeData[1][i] = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];			//load raw data to strobe 0
		EngineData.u32StrobeData[0][i] = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];			//load raw data to strobe 1
		//calculate data more than h1~h5
		u32temp1 = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];
		u32temp1 &= EngineData.u32LineBuf[h1][i];					//bits burned in 1 line history
		u32temp2 = (u32temp1 & EngineData.u32LineBuf[h2][i]);		//bits burned in 2 line history
		u32temp3 = (u32temp2 & EngineData.u32LineBuf[h3][i]);		//bits burned in 3 line history
		u32temp4 = (u32temp3 & EngineData.u32LineBuf[h4][i]);		//bits burned in 4 line history
		u32temp5 = (u32temp4 & EngineData.u32LineBuf[h5][i]);		//bits burned in 5 line history
		u32temp1 &= ~u32temp2;							//clear h2 data over h1
		u32temp2 &= ~u32temp3;							//clear h3 data over h2
		u32temp3 &= ~u32temp4;							//clear h4 data over h3
		u32temp4 &= ~u32temp5;							//clear h5 data over h4
		//burn history before 1 blank line
		u32tempL1 = ~EngineData.u32LineBuf[h1][i];		//load blank dots in last line in
		u32tempL1 &= EngineData.u32LineBuf[h2][i];
		u32tempL1 &= EngineData.u32LineBuf[h3][i];				//2 line burned before last line off
		u32tempL2 = (u32tempL1 & EngineData.u32LineBuf[h4][i]);	//3 line burned before last line off
		u32tempL1 &= ~u32tempL2;						//clear L2 data over L1
		//burn history before 2 blank lines
		u32tempH1 = ~EngineData.u32LineBuf[h1][i];		//load blank dots in last line
		u32tempH1 &= (~EngineData.u32LineBuf[h2][i]);	//blank dots both in last 2 lines
		u32tempH1 &= EngineData.u32LineBuf[h3][i];
		u32tempH1 &= EngineData.u32LineBuf[h4][i];				//2 line burned before last 2 lines off
		u32tempH2 = (u32tempH1 & EngineData.u32LineBuf[h5][i]);	//3 line burned before last 2 lines off
		u32tempH1 &= ~u32tempH2;						//clear L2 data over L1;
		//next line will be blank or the line after next will be blank
		u32tempF1 = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];		//load raw data of current line
		u32tempF1 &= ~EngineData.u32LineBuf[f1][i];				//dots on current that will be off in next line;
		u32tempF2 = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];
		u32tempF2 &= (~ EngineData.u32LineBuf[f2][i]);				//dots on current that will be off in next 2 line;
		u32tempF2 &= ~u32tempF1;						//clear F2 data over F1;
		//heat compensate for lines history
		EngineData.u32StrobeData[3][i] &= ~u32temp1;		//1 line history
		//EngineData.u32StrobeData[2][i] &= ~u32temp1;
		//EngineData.u32StrobeData[1][i] &= ~u32temp1;
		//EngineData.u32StrobeData[0][i] &= ~u32temp1;
		//EngineData.u32StrobeData[3][i] &= ~u32temp2;		//2 lines continuous history
		//EngineData.u32StrobeData[2][i] &= ~u32temp2;
		//EngineData.u32StrobeData[1][i] &= ~u32temp2;
		//EngineData.u32StrobeData[0][i] &= ~u32temp2;
		//EngineData.u32StrobeData[3][i] &= ~u32temp3;		//3 lines continuous history
		//EngineData.u32StrobeData[2][i] &= ~u32temp3;
		//EngineData.u32StrobeData[1][i] &= ~u32temp3;
		EngineData.u32StrobeData[1][i] &= ~u32temp3;
		EngineData.u32StrobeData[1][i] &= ~u32temp4;		//4 lines continuous history
		//EngineData.u32StrobeData[1][i] &= ~u32temp4;
		EngineData.u32StrobeData[1][i] &= ~u32temp5;		//5 lines continuous history
		//EngineData.u32StrobeData[1][i] &= ~u32temp5;

		//heat compensate for over burn before 1 blank line
		EngineData.u32StrobeData[3][i] &= ~u32tempL1;
		//EngineData.u32StrobeData[2][i] &= ~u32tempL1;
		//EngineData.u32StrobeData[1][i] &= ~u32tempL1;
		//EngineData.u32StrobeData[0][i] &= ~u32tempL1;
		//EngineData.u32StrobeData[3][i] &= ~u32tempL2;
		//EngineData.u32StrobeData[2][i] &= ~u32tempL2;
		//EngineData.u32StrobeData[1][i] &= ~u32tempL2;
		EngineData.u32StrobeData[1][i] &= ~u32tempL2;
		// heat compensate for over burn before 2 blank lines,
		if(EngineData.u8Spd>FEED_5_IPS){
			//EngineData.u32StrobeData[3][i] &= ~u32tempH1;
			EngineData.u32StrobeData[3][i] &= ~u32tempH2;
		}
		// heat compensate for next line will be off;
		//EngineData.u32StrobeData[0][i] &= ~((u32temp2)&u32tempF1);
		EngineData.u32StrobeData[3][i] &= ~((u32temp3|u32temp4|u32temp5)&u32tempF1);
		//u32StrobeData[1][i] &= ~(u32temp&u32tempF1);
		EngineData.u32StrobeData[1][i] &= ~((u32temp4|u32temp5)&u32tempF1);
		// heat compensate for the line after next 2 lines will be off, only for 8IPS
		EngineData.u32StrobeData[3][i] &= ~(u32temp5&u32tempF2);

	}
	EngineData.u8LineBufPtr ++;
	EngineData.u8LineBufPtr &= 0x07;
}

#else
void Strobe_Data_Calc(uint32_t* u32NewLinePtr, uint16_t u16LineWidth){
	uint8_t i;
	uint8_t h1,h2,h3,h4,h5,f1,f2;
	uint32_t u32temp5,u32temp4,u32temp3,u32temp2,u32temp1;	//history 1 to 5
	uint32_t u32tempL1, u32tempL2;	//history before 1 blank line
	uint32_t u32tempH1, u32tempH2;	//history before 2 blank line
	uint32_t u32tempF1, u32tempF2;	//future lines
	uint16_t u16WordCount;
	u16WordCount = u16LineWidth/32;

	h1 = (EngineData.u8LineBufPtr+7) & 0x07;
	h2 = (EngineData.u8LineBufPtr+6) & 0x07;
	h3 = (EngineData.u8LineBufPtr+5) & 0x07;
	h4 = (EngineData.u8LineBufPtr+4) & 0x07;
	h5 = (EngineData.u8LineBufPtr+3) & 0x07;
	f1 = (EngineData.u8LineBufPtr+1) & 0x07;
	f2 = (EngineData.u8LineBufPtr+2) & 0x07;
	for(i=0;i<u16WordCount;i++){
		EngineData.u32LineBuf[f2][i] = u32NewLinePtr[i];	//load data to recent 8 lines
		EngineData.u32StrobeData[3][i] = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];	//load raw data to main strobe 3
		EngineData.u32StrobeData[2][i] = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];	//load raw data to main strobe 2
		EngineData.u32StrobeData[1][i] = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];			//load raw data to strobe 0
		EngineData.u32StrobeData[0][i] = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];			//load raw data to strobe 1
		//calculate data more than h1~h5
		u32temp1 = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];
		u32temp1 &= EngineData.u32LineBuf[h1][i];					//bits burned in 1 line history
		u32temp2 = (u32temp1 & EngineData.u32LineBuf[h2][i]);		//bits burned in 2 line history
		u32temp3 = (u32temp2 & EngineData.u32LineBuf[h3][i]);		//bits burned in 3 line history
		u32temp4 = (u32temp3 & EngineData.u32LineBuf[h4][i]);		//bits burned in 4 line history
		u32temp5 = (u32temp4 & EngineData.u32LineBuf[h5][i]);		//bits burned in 5 line history
		u32temp1 &= ~u32temp2;							//clear h2 data over h1
		u32temp2 &= ~u32temp3;							//clear h3 data over h2
		u32temp3 &= ~u32temp4;							//clear h4 data over h3
		u32temp4 &= ~u32temp5;							//clear h5 data over h4
		//burn history before 1 blank line
		u32tempL1 = ~EngineData.u32LineBuf[h1][i];		//load blank dots in last line in
		u32tempL1 &= EngineData.u32LineBuf[h2][i];
		u32tempL1 &= EngineData.u32LineBuf[h3][i];				//2 line burned before last line off
		u32tempL2 = (u32tempL1 & EngineData.u32LineBuf[h4][i]);	//3 line burned before last line off
		u32tempL1 &= ~u32tempL2;						//clear L2 data over L1
		//burn history before 2 blank lines
		u32tempH1 = ~EngineData.u32LineBuf[h1][i];		//load blank dots in last line
		u32tempH1 &= (~EngineData.u32LineBuf[h2][i]);	//blank dots both in last 2 lines
		u32tempH1 &= EngineData.u32LineBuf[h3][i];
		u32tempH1 &= EngineData.u32LineBuf[h4][i];				//2 line burned before last 2 lines off
		u32tempH2 = (u32tempH1 & EngineData.u32LineBuf[h5][i]);	//3 line burned before last 2 lines off
		u32tempH1 &= ~u32tempH2;						//clear L2 data over L1;
		//next line will be blank or the line after next will be blank
		u32tempF1 = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];		//load raw data of current line
		u32tempF1 &= ~EngineData.u32LineBuf[f1][i];				//dots on current that will be off in next line;
		u32tempF2 = EngineData.u32LineBuf[EngineData.u8LineBufPtr][i];
		u32tempF2 &= (~ EngineData.u32LineBuf[f2][i]);				//dots on current that will be off in next 2 line;
		u32tempF2 &= ~u32tempF1;						//clear F2 data over F1;
		//heat compensate for lines history
		EngineData.u32StrobeData[3][i] &= ~u32temp1;		//1 line history
		//EngineData.u32StrobeData[2][i] &= ~u32temp1;
		//EngineData.u32StrobeData[1][i] &= ~u32temp1;
		//EngineData.u32StrobeData[0][i] &= ~u32temp1;
		//EngineData.u32StrobeData[3][i] &= ~u32temp2;		//2 lines continuous history
		//EngineData.u32StrobeData[2][i] &= ~u32temp2;
		//EngineData.u32StrobeData[1][i] &= ~u32temp2;
		//EngineData.u32StrobeData[0][i] &= ~u32temp2;
		//EngineData.u32StrobeData[3][i] &= ~u32temp3;		//3 lines continuous history
		//EngineData.u32StrobeData[2][i] &= ~u32temp3;
		//EngineData.u32StrobeData[1][i] &= ~u32temp3;
		EngineData.u32StrobeData[0][i] &= ~u32temp3;
		EngineData.u32StrobeData[0][i] &= ~u32temp4;		//4 lines continuous history
		//EngineData.u32StrobeData[1][i] &= ~u32temp4;
		EngineData.u32StrobeData[0][i] &= ~u32temp5;		//5 lines continuous history
		//EngineData.u32StrobeData[1][i] &= ~u32temp5;

		//heat compensate for over burn before 1 blank line
		EngineData.u32StrobeData[3][i] &= ~u32tempL1;
		//EngineData.u32StrobeData[2][i] &= ~u32tempL1;
		//EngineData.u32StrobeData[1][i] &= ~u32tempL1;
		//EngineData.u32StrobeData[0][i] &= ~u32tempL1;
		//EngineData.u32StrobeData[3][i] &= ~u32tempL2;
		//EngineData.u32StrobeData[2][i] &= ~u32tempL2;
		//EngineData.u32StrobeData[1][i] &= ~u32tempL2;
		EngineData.u32StrobeData[0][i] &= ~u32tempL2;
		// heat compensate for over burn before 2 blank lines,
		if(EngineData.u8Spd>FEED_5_IPS){
			//EngineData.u32StrobeData[3][i] &= ~u32tempH1;
			EngineData.u32StrobeData[3][i] &= ~u32tempH2;
		}
		// heat compensate for next line will be off;
		EngineData.u32StrobeData[0][i] &= ~((u32temp2)&u32tempF1);
		EngineData.u32StrobeData[1][i] &= ~((u32temp3|u32temp4|u32temp5)&u32tempF1);
		//u32StrobeData[1][i] &= ~(u32temp&u32tempF1);
		EngineData.u32StrobeData[3][i] &= ~((u32temp4|u32temp5)&u32tempF1);
		// heat compensate for the line after next 2 lines will be off, only for 8IPS
		EngineData.u32StrobeData[3][i] &= ~(u32temp5&u32tempF2);

	}
	EngineData.u8LineBufPtr ++;
	EngineData.u8LineBufPtr &= 0x07;
}
#endif
/**
  * @brief  Perform Next label loading in to Engine Data from Print Task
  * @param	None
  * @retval None
  */
void Load_Next_Label(void){
	uint8_t i;
	if(PrintTask.u32LabelCounter){
		PrintTask.u8SetSpd = FEED_6_IPS;
		//PrintTask.u8SensorType = USE_GAP_SENSOR;
		PrintTask.u32LabelCounter--;
		EngineData.u16State |= (PrintTask.u16Status & (TASK_IMAGEPRINT|TASK_SENSECURVE|TASK_PIXELGRAPHIC|TASK_FENCEGRAPHIC));
		EngineData.u8Darkness = PrintTask.u8SetDensity;
		EngineData.u32StepCount = PrintTask.u32LabelLength;
		EngineData.u32PrintLineCount = PrintTask.u32LabelLength;
		EngineData.u16PrintLineWidth = PrintTask.u16LabelWidth;
		PrintTask.u32LabelLength = 0;
		EngineData.u8DataPtr = PrintTask.u8DataHeadPtr;
		EngineData.u16State |= (ENGINE_PRINTON|ENGINE_USEGAP);
		if(PrintTask.u32LabelCounter == 0){
			PrintTask.u16Status = TASK_EMPTY;
		}
		for(i=0;i<2;i++){
			Strobe_Data_Calc((uint32_t*)(EngineData.u8DataPtr),EngineData.u16PrintLineWidth);
			if(EngineData.u16State & ENGINE_IMAGEPRINT){
				EngineData.u8DataPtr += (EngineData.u16PrintLineWidth/8);
			}
		}
		if(DebugSwitch & PH_SPI_ON){
			//Latch data for first line
			HAL_SPI_Transmit_DMA(&hspiPH, (uint8_t*)&EngineData.u32StrobeData[0][0], 104);//(EngineData.u16PrintLineWidth/8));
		}
	}
	else{
		EngineData.u16State &= ~(ENGINE_IMAGEPRINT);
		if(EngineData.u16State & (ENGINE_SENSECURVE|ENGINE_PIXELGRAPHIC|ENGINE_FENCEGRAPHIC)){
			EngineData.u32PrintLineCount = 0;
			EngineData.u16PrintLineWidth = 0;
			EngineData.u16State &= ~(ENGINE_SENSECURVE|ENGINE_PIXELGRAPHIC|ENGINE_FENCEGRAPHIC);
		}
	}
}
