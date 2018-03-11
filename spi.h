/*
 * spi.h
 *
 *  Created on: Dec 20, 2017
 *      Author: pc
 */

#ifndef BSP_HAL_INC_SPI_H_
#define BSP_HAL_INC_SPI_H_

#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "misc.h"
#include "stdint.h"
#include "stm32f10x.h"


#define GYRO_SPI           SPI3
#define SPI_GPIO           GPIOB
#define CS_GPIO            GPIOD
#define SPI_RCC_CLK        RCC_APB1Periph_SPI3
#define SPI_GPIO_CLK       RCC_APB2Periph_GPIOB
#define SPI_DMA            DMA2
#define SPI_DMA_Rx_CHANNEL DMA2_Channel1
#define SPI_DMA_Tx_CHANNEL DMA2_Channel2
#define SPIDMA_BUFFER_SIZE 20
#define SPI_DMA_CLK        RCC_AHBPeriph_DMA2
#define SCK          	   GPIO_Pin_3
#define MISO               GPIO_Pin_4
#define MOSI               GPIO_Pin_5
#define CE                 GPIO_Pin_12
#define CS                 GPIO_Pin_2

void spi_init(void);
void SPI_SendBytes(uint8_t* data,uint8_t Addr,uint16_t ln);
void SPI_ReceiveBytes(int8_t* data,uint8_t Addr,uint16_t ln);

#endif /* BSP_HAL_INC_SPI_H_ */
