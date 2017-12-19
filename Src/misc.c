/*
 * misc.c
 *
 *  Created on: Oct 27, 2017
 *      Author: KLam
 */

/* Includes -------------------------------------------------------------*/
#include "misc.h"
#include "thermal_print.h"
#include "comm_handle.h"
#include "test_mod.h"
#include "sensor_handle.h"
#include "mem_handle.h"
/* Variables Definitions ------------------------------------------------*/
uint16_t u16SliceCounter;
uint16_t u16TimerSlotCounter;
uint32_t u32ADTemp[3][6];
uint32_t u32ADFiltered[6];
uint8_t u8LEDState;

//for Debug Only
uint32_t DebugSwitch;

FMC_SDRAM_CommandTypeDef SDRAMcommand;

/* Function Definitions -------------------------------------------------*/
/**
  * @brief  Perform the SliceCounter for RTOS
  * @param  None
  * @param  None
  * @retval None
  */
void System_Tick_ISR(void) {
	uint16_t u16RTOS_Temp;
	u16RTOS_Temp = u16SliceCounter;
	u16SliceCounter += 1;
	u16TimerSlotCounter |= ((u16RTOS_Temp ^ u16SliceCounter) + 1);
	//HAL_GPIO_TogglePin(PH_POWER_OFF_GPIO_Port, PH_POWER_OFF_Pin);	//watch dog for PH power
}

/**
  * @brief  Perform ADC switching Sampling, called in AD DMA IRQ
  * @param  None
  * @param  None
  * @retval None
  */
void ADC_Filter_ISR(void)
{
	static uint8_t u8Counter=0;
	static uint8_t u8WDCounter=1;
	uint8_t i,j;
	//HAL_GPIO_WritePin(LED_RED_N_GPIO_Port,LED_RED_N_Pin,GPIO_PIN_SET);
	if(u8Counter<7){
		u8Counter++;
	}
	else{
		u8Counter = 0;
		//HAL_GPIO_TogglePin(LED_RED_N_GPIO_Port,LED_RED_N_Pin);
		for(i=0;i<6;i++){
			//u32ADFiltered[i] = 0;
			for(j=0;j<3;j++){
				u32ADFiltered[i] += u32ADTemp[j][i];
			}
			u32ADFiltered[i] >>=2;

		}
		if(u8WDCounter){
			u8WDCounter--;
			return;
		}
		u8WDCounter = 1;
		if(u32ADFiltered[ADQ_PHTEMP] > u16PHTempTblM[20]){
			//temperature < 80C
#ifdef	DEBUG_IS_ON
			if(DebugSwitch & PH_WD_ON)
#else
#endif
			{
				HAL_GPIO_TogglePin(PH_POWER_OFF_GPIO_Port, PH_POWER_OFF_Pin);	//watch dog for PH power
			}
		}
	}
	//HAL_GPIO_WritePin(LED_RED_N_GPIO_Port,LED_RED_N_Pin,GPIO_PIN_RESET);
}

/**
  * @brief  Perform ADC switching Sampling
  * @param  None
  * @param  None
  * @retval None
  */
int32_t MCU_Temp_Calc(uint32_t* u32ADPtr){
	int32_t s32Temp;
	//HAL_GPIO_WritePin(LED_RED_N_GPIO_Port,LED_RED_N_Pin,GPIO_PIN_SET);
	s32Temp = (int16_t) (*u32ADPtr) - 943;
	s32Temp /= 3;
	s32Temp += 25;
	//HAL_GPIO_WritePin(LED_RED_N_GPIO_Port,LED_RED_N_Pin,GPIO_PIN_RESET);
	return s32Temp;
}

/**
  * @brief  Perform LED Rreshing, called very 4 ms after Key Sampling
  * @param  None
  * @param  None
  * @retval None
  */
void LED_Rreshing(void){
	static uint8_t u8PrevLEDState;
	static uint8_t u8BlinkCounter;
	static uint8_t u8BlinkTimeCount;
	if(u8LEDState){
		if(u8LEDState<7){
			if(u8LEDState != u8PrevLEDState){
				u8BlinkCounter = (u8LEDState<<1);
				u8BlinkTimeCount = 15;
				HAL_GPIO_WritePin(LED_RED_N_GPIO_Port, LED_RED_N_Pin, GPIO_PIN_RESET);
			}else{
				if(u8BlinkCounter){
					if(u8BlinkTimeCount >0){
						u8BlinkTimeCount --;
					}else{
						u8BlinkTimeCount = 15;
						u8BlinkCounter --;
						if(u8BlinkCounter){
						HAL_GPIO_TogglePin(LED_RED_N_GPIO_Port,LED_RED_N_Pin);
						}
					}
				}
			}
			HAL_GPIO_WritePin(LED_GRN_N_GPIO_Port, LED_GRN_N_Pin, GPIO_PIN_RESET);
		}
		else{
			//error state
			HAL_GPIO_WritePin(LED_RED_N_GPIO_Port, LED_RED_N_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_GRN_N_GPIO_Port, LED_GRN_N_Pin, GPIO_PIN_SET);
		}
	}else{
		//State = 0, Green
		HAL_GPIO_WritePin(LED_GRN_N_GPIO_Port, LED_GRN_N_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_RED_N_GPIO_Port, LED_RED_N_Pin, GPIO_PIN_SET);
	}
	u8PrevLEDState = u8LEDState;
}

/**
  * @brief  Perform Feed Key sampling, called very 4 ms
  * @param  None
  * @param  None
  * @retval None
  */
void Feed_Key_Sampling(void){
	static uint32_t u32PressTimeCounter;
	static GPIO_PinState DeditherKeyState = GPIO_PIN_SET;
	static GPIO_PinState PrevKeyState = GPIO_PIN_SET;
	static uint8_t u8DitherCounter;
	static uint8_t j;
	static uint32_t u32Prev24V;
	uint16_t u16Length;
	uint8_t i;
	uint8_t cmd_buf[20];
	if(DeditherKeyState == HAL_GPIO_ReadPin(FEEDKEY_N_GPIO_Port, FEEDKEY_N_Pin)){
		u8DitherCounter ++;
		if(u8DitherCounter > 2){
			u8DitherCounter = 3;
			if(DeditherKeyState == GPIO_PIN_RESET){
				//Key is pressed
				if(PrevKeyState == GPIO_PIN_RESET){
					//Key is pressed and hold
					if(u32PressTimeCounter < 2500){		//< 10 seconds
						u32PressTimeCounter++;
					}
					for(i=10;i>0;i--){
						if(u32PressTimeCounter>(250*i-1)){
							break;
						}
					}
					if(j != i){
						if(i<7){
						u8LEDState= i;}
#ifdef DEBUG_IS_ON
						if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
							sprintf((*CurrLogPtr).cdataptr,"Key is pressed for %d secs\r\n", i);
							u16Length = strlen((*CurrLogPtr).cdataptr);
							(*CurrLogPtr).u16Length += u16Length;
							(*CurrLogPtr).cdataptr += u16Length;
						}
#endif
					}
					j=i;
				}
				else{
					//Key is just pressed
				}
			}
			else{
				if(PrevKeyState == GPIO_PIN_RESET){
					//Key is just released
					for(i=10;i>0;i--){
						if(u32PressTimeCounter>(250*i-1)){
							break;
						}
					}
					switch (i){
					case 0:{
						//feed 1 label
						if(EngineData.u16State & ENGINE_READY){
							sprintf((char*)cmd_buf,"set.1\r\n");
							Data_Handle(cmd_buf, 7);
						}else{
							EngineData.u16State |= ENGINE_ERROR;
#ifdef DEBUG_IS_ON
							if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
								sprintf((*CurrLogPtr).cdataptr,"Emergency Stop!\r\n");
								u16Length = strlen((*CurrLogPtr).cdataptr);
								(*CurrLogPtr).u16Length += u16Length;
								(*CurrLogPtr).cdataptr += u16Length;
							}
#endif
						}
					}break;
					case 1:{
						//print threshold
						sprintf((char*)cmd_buf,"set.15:12\r\n");
						Data_Handle(cmd_buf, 11);
						sprintf((char*)cmd_buf,"set.16:1\r\n");
						Data_Handle(cmd_buf, 10);
					}break;
					case 2:{
						sprintf((char*)cmd_buf,"set.20\r\n");
						Data_Handle(cmd_buf, 8);
					}break;
					case 3:{
						//calibration
						sprintf((char*)cmd_buf,"set.21\r\n");
						Data_Handle(cmd_buf, 8);
					}break;
					case 5:{
						sprintf((char*)cmd_buf,"set.14:500\r\n");
						Data_Handle(cmd_buf, 12);
					}break;
					default:{
#ifdef DEBUG_IS_ON
						if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
							sprintf((*CurrLogPtr).cdataptr,"Key is released\r\n");
							u16Length = strlen((*CurrLogPtr).cdataptr);
							(*CurrLogPtr).u16Length += u16Length;
							(*CurrLogPtr).cdataptr += u16Length;
						}
#endif

					}break;
					}
					u8LEDState = 0;
				}
				else{
					//Key is no pressed
				}
				u32PressTimeCounter = 0;
			}
			PrevKeyState = DeditherKeyState;
		}
	}
	else{
		DeditherKeyState = HAL_GPIO_ReadPin(FEEDKEY_N_GPIO_Port, FEEDKEY_N_Pin);
		u8DitherCounter = 0;
	}
	if(u32ADFiltered[ADQ_P24V] < 1700){
		if(u32Prev24V>=1700){
#ifdef DEBUG_IS_ON
			if((*CurrLogPtr).u16Length < LOG_BUF_SIZE){
				sprintf((*CurrLogPtr).cdataptr,"Shutting Down...\r\n");
				u16Length = strlen((*CurrLogPtr).cdataptr);
				(*CurrLogPtr).u16Length += u16Length;
				(*CurrLogPtr).cdataptr += u16Length;
			}
#endif
		}
	}
	u32Prev24V = u32ADFiltered[ADQ_P24V];
}
