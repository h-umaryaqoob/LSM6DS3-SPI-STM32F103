/*
 * main.c
 *
 *  Created on: Sep 24, 2017
 *      Author: muaz waseem
 */

#include "main.h"

uint32_t i = 0,distance;
LSM6DS3_IMUData AngleRate;
LSM6DS3_IMUData AccRate;
LSM6DS3_EulerAngles angles;
RawData testingData;
int8_t dataxl[7];
float yangle,xl_x,xl_y,xl_z,g_x,g_y,g_z;

int main(void)
{
	SysTick_Init();
	LSM6DS3_init();

	who_am_i();
	while(1)
	{
/*
//		nyaTriqa();
		LSM6DS3_ReadAcc(&AccRate);
		//readRawValues(dataxl);
		//testinStruc0tMethod(&testingData);
		xl_x = (float)AccRate.Data_X*AccRate.Senstivity;
		xl_y = (float)AccRate.Data_Y*AccRate.Senstivity;
		xl_z = (float)AccRate.Data_Z*AccRate.Senstivity;
		//getInclinationAngleY(&yangle);

		LSM6DS3_ReadAngularRate(&AngleRate);
		g_x = (float)AngleRate.Data_X*AngleRate.Senstivity;
		g_y = (float)AngleRate.Data_Y*AngleRate.Senstivity;
		g_z = (float)AngleRate.Data_Z*AngleRate.Senstivity;

*/
		getInclinationAngleY(&yangle);
		delay_Nus(125);
	}
	return 0;
}

void fatal_error_call_back(int8_t cause)
{
	for( ;; );
}


/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

void delay_x_ticks(uint32_t x)
{
	uint32_t start_tick = xTaskGetTickCount();
	uint32_t tick_count;

	do
	{
		if(xTaskGetTickCount() >= start_tick)
		{
			tick_count = xTaskGetTickCount() - start_tick;
		}
		else
		{
			tick_count = ((0xFFFFFFFFUL) - start_tick) + xTaskGetTickCount();
		}

	}while(tick_count < x);
}

void HardFault_Handler(void)
{
	//hard_power_off();
	while(1)
	{}
}
