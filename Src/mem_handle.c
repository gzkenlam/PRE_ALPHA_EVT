/*
 * mem_handle.c
 *
 *  Created on: Nov 1, 2017
 *      Author: KLam
 */
#include "mem_handle.h"

//uint8_t u8MemStatus=0;
//uint32_t u32Page1RcvLength;
//uint32_t u32Page2RcvLength;

SDRAM_BUF_TYPE SDRAMBuf[SDRAM_BUF_PAGES];
uint8_t u8WrBufSeq;
uint8_t u8RdBufSeq;

/**
  * @brief  Perform the SDRAM buff initial
  * @param  None
  * @param  None
  * @retval None
  */
void SDRAM_Buff_Init(void){
	uint8_t i;
	for(i=0;i<SDRAM_BUF_PAGES;i++){
		SDRAMBuf[i].cRAMPtr = (char*)(SDRAM_BANK_ADDR + SDRAM_BUF_OFFSET + SDRAM_BUF_PAGESIZE*i);
		SDRAMBuf[i].u32Length = 0;
		SDRAMBuf[i].u8State = RAM_IDLE;
	}
	u8WrBufSeq = 0;
	u8RdBufSeq = 0;
}

/**
  * @brief  Perform the SDRAM exernal memory inialization sequence
  * @param  hsdram: SDRAM handle
  * @param  Command: Pointer to SDRAM command structure
  * @retval None
  */
void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
  __IO uint32_t tmpmrd =0;
  /* Step 3:  Configure a clock configuration enable command */
  Command->CommandMode 			 = FMC_SDRAM_CMD_CLK_ENABLE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 4: Insert 100 ms delay */
  HAL_Delay(100);

  /* Step 5: Configure a PALL (precharge all) command */
  Command->CommandMode 			 = FMC_SDRAM_CMD_PALL;
  Command->CommandTarget 	     = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 6 : Configure a Auto-Refresh command */
  Command->CommandMode 			 = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber 	 = 4;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 7: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_2          |
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                     SDRAM_MODEREG_CAS_LATENCY_3           |
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command->CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 8: Set the refresh rate counter */
  /* (15.62 us x Freq) - 20 */
  /* Set the device refresh counter */
  HAL_SDRAM_ProgramRefreshRate(hsdram, REFRESH_COUNT);
}

/**
  * @brief  Perform the SDRAM exernal memory verfication sequence
  * @param	hdma: MEM to MEM DMA handle
  * @param	ScrAddr: source address
  * @param	TagAddr: targer address
  * @length	length: test string length
  * @retval 0 success or 1 fail
  */
uint8_t SDRAM_Verification(DMA_HandleTypeDef *hdma,uint32_t *u32ScrAddr, uint32_t *u32TagAddr, uint32_t u32Length){
	uint32_t i;
	if(HAL_DMA_Start_IT(hdma,(uint32_t)(u32ScrAddr),(uint32_t)(u32TagAddr),u32Length) == HAL_OK){
		while(HAL_DMA_GetState(hdma)==HAL_DMA_STATE_BUSY);
		for(i=0;i<u32Length;i++){
			if(*u32ScrAddr != *u32TagAddr){
				return 1;
			}
			u32ScrAddr++;
			u32TagAddr++;
		}
		return 0;
	}
	else{
		return 1;
	}
}
