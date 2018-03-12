/*
 * main.h
 *
 *  Created on: Sep 24, 2017
 *      Author: Umar Yaqoob
 */

#ifndef APPLICATION_MAIN_H_
#define APPLICATION_MAIN_H_

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
#include "stdint.h"
#include "usart.h"
#include "lsm6ds3.h"
#include "delay.h"


void delay_x_ticks(uint32_t x);
void fatal_error_call_back(int8_t cause);

#endif /* APPLICATION_MAIN_H_ */
