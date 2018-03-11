/*
 * delay.c
 *
 *  Created on: Jan 30, 2018
 *      Author: Umar Yaqoob
 */
#include "delay.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
__IO uint32_t sysTickCounter;

void SysTick_Init(void) {
	/****************************************
	 *SystemFrequency/1000      1ms         *
	 *SystemFrequency/100000    10us        *
	 *SystemFrequency/1000000   1us         *
	 *****************************************/
	while (SysTick_Config(SystemCoreClock / 1000000) != 0)
	{}  // One SysTick interrupt now equals 1us

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpioStructure;

    gpioStructure.GPIO_Pin = GPIO_Pin_7;
    gpioStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpioStructure);

    gpioStructure.GPIO_Pin = GPIO_Pin_8;
    gpioStructure.GPIO_Mode = GPIO_Mode_IPD;//GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpioStructure);

}

// This method needs to be called in the SysTick_Handler:

void TimeTick_Decrement(void)
{
	if(sysTickCounter != 0x00)
	{
		sysTickCounter--;
	}
}

void delay_Nus(u32 n)
{
	sysTickCounter = n;
	while(sysTickCounter != 0){}
}

void delay_1ms(void)
{
	sysTickCounter = 1000;
	while(sysTickCounter != 0){}
}

void delay_Nms(u32 n)
{
	while(n--)
	{
		delay_1ms();
	}
}

uint32_t Read_Distance(void)
{
	__IO uint8_t flag=0;
	__IO uint32_t disTime=0;
	GPIO_SetBits(GPIOA,GPIO_Pin_7);
	delay_Nus(10);
	GPIO_ResetBits(GPIOA,GPIO_Pin_7);

	while(flag == 0)
	{
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8) == SET)
		{
			disTime++;
			flag = 1;
		}

	}
	return disTime;
}

void SysTick_Handler(void)
{
	TimeTick_Decrement();
}

