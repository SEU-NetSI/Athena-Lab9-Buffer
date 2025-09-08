/*
 * w25q64_ll.c
 *
 *  Created on: May 20, 2024
 *      Author: mfxjx
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "w25q64_ll.h"
#include "spi_drv.h"

#define SPI_BUFFER_MAX_SIZE 240
static uint8_t spiTxBuffer[SPI_BUFFER_MAX_SIZE];
static uint8_t spiRxBuffer[SPI_BUFFER_MAX_SIZE];

void W25Qx_Enable_DMA()
{
	spiBeginTransaction();
	LL_GPIO_ResetOutputPin(W25QXX_CS_GPIO_Port, W25QXX_CS_Pin);
}

void W25Qx_Disable_DMA()
{
	LL_GPIO_SetOutputPin(W25QXX_CS_GPIO_Port, W25QXX_CS_Pin);
	spiEndTransaction();
}

void spiRead(const void* cmd,
			size_t cmdLength,
			void *data,
			size_t dataLength,
			SPI_TypeDef* SPIx)
{
	spiBeginTransaction();
	W25Qx_Enable();
	memcpy(spiTxBuffer, cmd, cmdLength);
	memset(spiTxBuffer + cmdLength, DUMMY_BYTE, dataLength);
	spiExchange(SPIx, cmdLength + dataLength, spiTxBuffer, spiRxBuffer);
	memcpy(data, spiRxBuffer + cmdLength, dataLength);
	W25Qx_Disable();
	spiEndTransaction();
}


uint8_t BSP_W25Qx_Init(void)
{
	BSP_W25Qx_Reset();
	return BSP_W25Qx_GetStatus();
}


static void	BSP_W25Qx_Reset(void)
{
	uint8_t cmd[2]= {RESET_ENABLE_CMD,RESET_MEMORY_CMD};
	uint8_t dummy[]= {DUMMY_BYTE};
	spiRead(cmd, 2, dummy, 1, SPI3);
	vTaskDelay(1);
}


static uint8_t BSP_W25Qx_GetStatus(void)
{
	uint8_t cmd[] = {READ_STATUS_REG1_CMD};
	uint8_t status= 0xFF;
	spiRead(cmd, 1, &status, 1, SPI3);

	if((status & W25Q128FV_FSR_BUSY) != 0)
	{
		return W25Qx_BUSY;
	}
	else
	{
		return W25Qx_OK;
	}
}

/**********************************************************************************
 * 函数功能: 获取设备ID
 * 待补充
 */
void BSP_W25Qx_Read_ID(uint8_t *ID)
{

}

/**********************************************************************************
 * 函数功能: 写使能
 */
uint8_t BSP_W25Qx_WriteEnable(void)
{
	uint8_t cmd[] = {WRITE_ENABLE_CMD};
	/*Select the FLASH: Chip Select low */
	W25Qx_Enable_DMA();
	/* Send the read ID command */
	uint8_t tmpdummy[1]={0x00};
	spiExchange(SPI3, 1, cmd, tmpdummy);
	/*Deselect the FLASH: Chip Select high */
	W25Qx_Disable_DMA();
	return W25Qx_OK;
}

/**********************************************************************************
 * 函数功能: 读数据
 * 输入参数: 缓存数组指针、读地址、字节数
 * 待补充
 */
uint8_t BSP_W25Qx_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{

}

/**********************************************************************************
  * 函数功能: 写数据
  * 输入参数: 缓存数组指针、写地址、字节数
  * 待补充
  */
uint8_t BSP_W25Qx_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{

}

/**********************************************************************************
  * 函数功能: 扇区擦除
  * 输入参数: 地址
  * 待补充
  */
uint8_t BSP_W25Qx_Erase_Block(uint32_t Address)
{

}

 /**********************************************************************************
  * 函数功能: 芯片擦除
  */
uint8_t BSP_W25Qx_Erase_Chip(void)
{
	uint8_t cmd[4];
	cmd[0] = CHIP_ERASE_CMD;

	/* Enable write operations */
	BSP_W25Qx_WriteEnable();

	/*Select the FLASH: Chip Select low */
	W25Qx_Enable_DMA();
	/* Send the read ID command */
	spiExchange(SPI3, 1, cmd, spiRxBuffer);
	/*Deselect the FLASH: Chip Select high */
	W25Qx_Disable_DMA();

	/* Wait the end of Flash writing */
	while(BSP_W25Qx_GetStatus() == W25Qx_BUSY);
	return W25Qx_OK;
}
