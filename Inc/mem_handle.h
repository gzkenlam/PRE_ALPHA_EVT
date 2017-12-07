/*
 * mem_handle.h
 *
 *  Created on: Nov 1, 2017
 *      Author: KLam
 */

#ifndef MEM_HANDLE_H_
#define MEM_HANDLE_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/* Macro Definitions ---------------------------------------------------------*/
#define SDRAM_FREQ			((uint32_t)42700000/100)
#define SDRAM_BANK_ADDR     ((uint32_t)0xC0000000)
#define WRITE_READ_ADDR     ((uint32_t)0x0000)
#define SDRAM_SIZE			((uint32_t)0x3ffff)
#define PAGE_SIZE			((uint32_t)0x3ffff)
#define	SDRAM_PAGE1_ADDR	((uint32_t)0xC0040000)
#define SDRAM_PAGE2_ADDR	((uint32_t)0xC0080000)
#define REFRESH_COUNT       ((uint32_t)(1526*SDRAM_FREQ-20))   /* SDRAM refresh counter (64MHz SDRAM clock) 15.26u*Freq-20 */
#define SDRAM_TIMEOUT     ((uint32_t)0xFFFF)
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

#define MEM_PAGE1_WRITING	((uint8_t)0x01)
#define MEM_PAGE1_WR_DONE	((uint8_t)0x02)
#define MEM_PAGE2_WRITING	((uint8_t)0x04)
#define MEM_PAGE2_WR_DONE	((uint8_t)0x08)
#define MEM_PAGE1_READING	((uint8_t)0x10)
#define MEM_PAGE1_RD_DONE	((uint8_t)0x20)
#define MEM_PAGE2_READING	((uint8_t)0x40)
#define MEM_PAGE2_RD_DONE	((uint8_t)0x80)

/* Exported Variables --------------------------------------------------------*/
extern	FMC_SDRAM_CommandTypeDef SDRAMcommand;
extern const uint8_t u8Image[1248][104];
extern	uint8_t u8MemStatus;
extern	uint32_t u32Page1RcvLength;
extern	uint32_t u32Page2RcvLength;

#define SDRAM_BUF_PAGES 16
typedef struct{
	char* cRAMPtr;
	uint32_t u32Length;
	uint8_t u8State;
}SDRAM_BUF_TYPE;
extern	SDRAM_BUF_TYPE SDRAMBuf[SDRAM_BUF_PAGES];
extern uint8_t u8WrBufSeq;
extern uint8_t u8RdBufSeq;
#define SDRAM_BUF_OFFSET	((uint32_t)0x40000)
#define SDRAM_BUF_PAGESIZE	((uint32_t)0x40000)
#define RAM_IDLE	(0)
#define RAM_WRITING (1)
#define RAM_WR_DONE	(2)
#define RAM_READING (3)

/* Exported Functions --------------------------------------------------------*/
uint8_t SDRAM_Verification(DMA_HandleTypeDef *hdma,uint32_t *u32ScrAddr, uint32_t *u32TagAddr, uint32_t u32Length);
void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command);
void SDRAM_Buff_Init(void);



#endif /* MEM_HANDLE_H_ */

