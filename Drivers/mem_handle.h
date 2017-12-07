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
#define SDRAM_BANK_ADDR     ((uint32_t)0xC0000000)
#define WRITE_READ_ADDR     ((uint32_t)0x0000)
#define SDRAM_SIZE			((uint32_t)0x3ffff)
#define REFRESH_COUNT       ((uint32_t)980)   /* SDRAM refresh counter (64MHz SDRAM clock) 15.26u*Freq-20 */
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

/* Exported Variables --------------------------------------------------------*/
extern	FMC_SDRAM_CommandTypeDef SDRAMcommand;
extern const uint8_t u8Image[1248][104];
/* Exported Functions --------------------------------------------------------*/
uint8_t SDRAM_Verification(DMA_HandleTypeDef *hdma,uint32_t *u32ScrAddr, uint32_t *u32TagAddr, uint32_t u32Length);
void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command);




#endif /* MEM_HANDLE_H_ */
