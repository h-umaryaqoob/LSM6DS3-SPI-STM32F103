/*
 * main.c
 *
 *  Created on: Sep 24, 2017
 *      Author: Umar Yaqoob
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

