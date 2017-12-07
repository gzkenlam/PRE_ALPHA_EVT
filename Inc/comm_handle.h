/*
 * comm_handle.h
 *
 *  Created on: Oct 31, 2017
 *      Author: KLam
 */

#ifndef COMM_HANDLE_H_
#define COMM_HANDLE_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Macro Definitions ---------------------------------------------------------*/
#define	TX_BUF_SIZE 64
#define RX_BUF_SIZE	64
/* Exported Variables --------------------------------------------------------*/
extern	UART_HandleTypeDef huart3;
extern	uint8_t u8TxBuf[];

typedef struct{
	char* cdataptr;
	int u16Length;
}LOG_DATA_TYPE;
#define LOG_PAGES 2
#define LOG_BUF_SIZE 256
extern LOG_DATA_TYPE DebugLog[LOG_PAGES];
extern LOG_DATA_TYPE *CurrLogPtr;


/* Exported Functions --------------------------------------------------------*/
void USB_Receive(void);
void Debug_Msg_Out(char* MsgPtr);
void LogBuf_Init(void);
uint32_t Data_Handle(uint8_t* u8RcvPtr, uint32_t u32Length);
#endif /* COMM_HANDLE_H_ */
