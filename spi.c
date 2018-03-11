/*
 * spi.c
 *
 *  Created on: Dec 20, 2017
 *      Author: pc
 */
#include "spi.h"

void spi_structure_init(void);
void spi_dma_init(void);
void spi_gpio_config(void);
void spi_rcc_config(void);
void SPIDMA_NVIC_Configuration(void);
void LSM6DS3_CS_LOW(void);
void LSM6DS3_CS_HIGH(void);
uint8_t IMU_Read[20];
//int shah;
//uint16_t oy = 4;

void spi_init(void)
{
	spi_rcc_config();
	spi_gpio_config();
	spi_structure_init();
	spi_dma_init();
	SPIDMA_NVIC_Configuration();

	SPI_I2S_DMACmd(GYRO_SPI, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(GYRO_SPI, SPI_I2S_DMAReq_Tx, ENABLE);

	DMA_ITConfig(SPI_DMA_Rx_CHANNEL,DMA_IT_TC,ENABLE);

	SPI_Cmd(GYRO_SPI, ENABLE);
	DMA_Cmd(SPI_DMA_Tx_CHANNEL, DISABLE);
	DMA_Cmd(SPI_DMA_Rx_CHANNEL, DISABLE);

	GPIO_SetBits(CS_GPIO,CS);
}

void spi_structure_init(void)
{
	SPI_InitTypeDef SPI_InitStructure;

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(GYRO_SPI, &SPI_InitStructure);
}

void spi_gpio_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_InitStructure.GPIO_Pin = SCK | MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(CS_GPIO, &GPIO_InitStructure);


}

void spi_rcc_config(void)
{
	RCC_PCLK2Config(RCC_HCLK_Div2);
	RCC_APB2PeriphClockCmd(SPI_GPIO_CLK |RCC_APB2Periph_GPIOD| RCC_APB2Periph_AFIO , ENABLE);//| RCC_APB2Periph_AFIO
	RCC_APB1PeriphClockCmd(SPI_RCC_CLK,ENABLE);
	RCC_AHBPeriphClockCmd(SPI_DMA_CLK, ENABLE);
}

void spi_dma_init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;

	DMA_DeInit(SPI_DMA_Rx_CHANNEL);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(GYRO_SPI->DR));
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(IMU_Read);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = SPIDMA_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(SPI_DMA_Rx_CHANNEL, &DMA_InitStructure);

	DMA_DeInit(SPI_DMA_Tx_CHANNEL);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(GYRO_SPI->DR));
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(IMU_Read);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = SPIDMA_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(SPI_DMA_Tx_CHANNEL, &DMA_InitStructure);

}


void LSM6DS3_CS_LOW(void)
{
	GPIO_ResetBits(CS_GPIO,CS);
}

void LSM6DS3_CS_HIGH(void)
{
	GPIO_SetBits(CS_GPIO,CS);
}

void DMA2_Channel1_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_IT_TC1) == SET)
	{
		LSM6DS3_CS_HIGH();
	}
	DMA_ClearITPendingBit(DMA2_IT_TC1);
}

void SPIDMA_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure and enable DMA interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void SPI_SendBytes(uint8_t* data,uint8_t Addr,uint16_t ln)
{
	uint8_t buffer[20] = {0};
	uint8_t i = 1;

	buffer[0] = Addr;
	for(;i<=ln;i++)
	{
		buffer[i] = data[i-1];
	}

	LSM6DS3_CS_LOW();
	DMA_Cmd(SPI_DMA_Tx_CHANNEL, DISABLE);
	DMA_Cmd(SPI_DMA_Rx_CHANNEL, DISABLE);

	SPI_DMA_Tx_CHANNEL->CNDTR = ln+1;
	SPI_DMA_Tx_CHANNEL->CMAR = (uint32_t)buffer;
	SPI_DMA_Rx_CHANNEL->CNDTR = ln+1;
	SPI_DMA_Rx_CHANNEL->CMAR = (uint32_t)buffer;

	DMA_Cmd(SPI_DMA_Tx_CHANNEL, ENABLE);
	DMA_Cmd(SPI_DMA_Rx_CHANNEL, ENABLE);
}

void SPI_ReceiveBytes(int8_t* data,uint8_t Addr,uint16_t ln)
{
	uint8_t buffer[12] = {0};
	buffer[0] = Addr|0x80;

	LSM6DS3_CS_LOW();
	DMA_Cmd(SPI_DMA_Tx_CHANNEL, DISABLE);
	DMA_Cmd(SPI_DMA_Rx_CHANNEL, DISABLE);

	SPI_DMA_Tx_CHANNEL->CNDTR = ln+1;
	SPI_DMA_Tx_CHANNEL->CMAR = (uint32_t)buffer;
	SPI_DMA_Rx_CHANNEL->CNDTR = ln+1;
	SPI_DMA_Rx_CHANNEL->CMAR = (uint32_t)data;

	DMA_Cmd(SPI_DMA_Tx_CHANNEL, ENABLE);
	DMA_Cmd(SPI_DMA_Rx_CHANNEL, ENABLE);
}
