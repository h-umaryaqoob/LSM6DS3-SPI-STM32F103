/*
 * delay.h
 *
 *  Created on: Jan 30, 2018
 *      Author: Umar
 */

#ifndef BSP_HAL_INC_DELAY_H_
#define BSP_HAL_INC_DELAY_H_

#include "stm32f10x.h"

void SysTick_Init(void);
void TimeTick_Decrement(void);
void delay_Nus(u32 n);
void delay_1ms(void);
void delay_Nms(u32 n);
uint32_t Read_Distance(void);
#endif /* BSP_HAL_INC_DELAY_H_ */
