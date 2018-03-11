/*
 * spi.c
 *
 *  Created on: Dec 6, 2017
 *      Author: pc
 */

#include "lsm6ds3.h"
#include "MahonyAHRS.h"
#include "spi.h"
#include <math.h>

#define SENSIVITY_XL_2g             ((float)0.061f)   /*mg/LSB*/
#define SENSIVITY_XL_4g             ((float)0.122f)   /*mg/LSB*/
#define SENSIVITY_XL_8g             ((float)0.244f)   /*mg/LSB*/
#define SENSIVITY_XL_16g            ((float)0.488f)   /*mg/LSB*/

#define SENSIVITY_G_125dps          ((float)4.375f)  /*mdps/LSB*/
#define SENSIVITY_G_250dps          ((float)8.75f)   /*mdps/LSB*/
#define SENSIVITY_G_500dps          ((float)17.50f)  /*mdps/LSB*/
#define SENSIVITY_G_1000dps         ((float)35.0f)   /*mdps/LSB*/
#define SENSIVITY_G_2000dps         ((float)70.0f)   /*mdps/LSB*/


float rate;                 // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
float Q_angle = 0.001f;     // Process noise variance for the accelerometer
float Q_bias = 0.003f;      // Process noise variance for the gyro bias
float R_measure = 0.0001f;//0.03f;    // Measurement noise variance - this is actually the variance of the measurement noise

float angle = 0.0f;   // Reset the angle
float bias = 0.0f;    // Reset bias

// Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix.
float A1 = 0.0f;
float A2 = 0.0f;
float A3 = 0.0f;
float A4 = 0.0f;

float Dt = 0.000125;


//---------------------------------------------------------------------------------------------------
// Definitions
/*
#define sampleFreq	512.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain
*/
//---------------------------------------------------------------------------------------------------
// Variable definitions
/*
volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
*/


int8_t i_am[2] = {0};
//int8_t tmpbuffer11[7] ={0};
int16_t tmp1;
int16_t tmp2;
int16_t tmp3;
RawData xlero;
float Accangle;

void LSM6DS3Gyro_Cmd(LSM6DS3Gyro_InitTypeDef* LSM6DS3Gyro_InitStructure);
void LSM6DS3XL_Cmd(LSM6DS3XL_InitTypeDef* LSM6DS3XL_InitStructure);
float getAngle(float newAngle, float newRate, float dt);
void bufferConcat(int8_t* buf1,int8_t* buf2,int16_t ln);


void LSM6DS3_init(void)
{
	LSM6DS3Gyro_InitTypeDef LSM6DS3Gyro_InitStructure;
	LSM6DS3XL_InitTypeDef   LSM6DS3XL_InitStructure;

	spi_init();
	LSM6DS3Gyro_InitStructure.BlockData_Update = BlockDataUpdate_Continous;
	LSM6DS3Gyro_InitStructure.Endianness = MSB_FIRST;
	LSM6DS3Gyro_InitStructure.Full_Scale = FS_G_1000dps;
	LSM6DS3Gyro_InitStructure.Output_DataRate = ODR_G_416Hz;
	LSM6DS3Gyro_InitStructure.Register_read = Register_read_continuous;
	LSM6DS3Gyro_Cmd(&LSM6DS3Gyro_InitStructure);

	LSM6DS3XL_InitStructure.Output_DataRate = ODR_XL_416Hz;
	LSM6DS3XL_InitStructure.Full_Scale = FS_XL_2g;
	LSM6DS3XL_InitStructure.BlockData_Update = BlockDataUpdate_Continous;
	LSM6DS3XL_InitStructure.Endianness = MSB_FIRST;
	LSM6DS3XL_InitStructure.BandWidth_Source = BandWidth_Default;
	LSM6DS3XL_InitStructure.Band_Width = BW_XL_100Hz;
	LSM6DS3XL_InitStructure.Register_read = Register_read_continuous;
	LSM6DS3XL_Cmd(&LSM6DS3XL_InitStructure);

}

void LSM6DS3Gyro_Cmd(LSM6DS3Gyro_InitTypeDef* LSM6DS3Gyro_InitStructure)
{
	uint8_t ctrl2 = 0x00;
	uint8_t ctrl3 = 0x00;

	ctrl2 |= (uint8_t)(LSM6DS3Gyro_InitStructure->Output_DataRate|LSM6DS3Gyro_InitStructure->Full_Scale);
	ctrl3 |= (uint8_t)(LSM6DS3Gyro_InitStructure->BlockData_Update|LSM6DS3Gyro_InitStructure->Endianness);
	ctrl3 |= (uint8_t)LSM6DS3Gyro_InitStructure->Register_read;

	SPI_SendBytes(&ctrl2,CTRL2_G_REG,1);
	SPI_SendBytes(&ctrl3,CTRL3_C_REG,1);

}

void LSM6DS3XL_Cmd(LSM6DS3XL_InitTypeDef* LSM6DS3XL_InitStructure)
{
	uint8_t ctrl1 = 0x00;
	uint8_t ctrl3 = 0x00;
	uint8_t ctrl4 = 0x00;

	ctrl1 |= (uint8_t)(LSM6DS3XL_InitStructure->Band_Width|LSM6DS3XL_InitStructure->Full_Scale|LSM6DS3XL_InitStructure->Output_DataRate);
	ctrl3 |= (uint8_t)(LSM6DS3XL_InitStructure->Endianness|LSM6DS3XL_InitStructure->BlockData_Update);
	ctrl4 |= (uint8_t)(LSM6DS3XL_InitStructure->BandWidth_Source);
	ctrl3 |= (uint8_t)LSM6DS3XL_InitStructure->Register_read;

	SPI_SendBytes(&ctrl1,CTRL1_XL_REG,1);
	SPI_SendBytes(&ctrl3,CTRL3_C_REG,1);
	SPI_SendBytes(&ctrl4,CTRL4_C_REG,1);
}

void who_am_i(void)
{
	SPI_ReceiveBytes(&i_am[0],WHO_AM_I_REG,1);//CTRL1_XL_REG,,WHO_AM_I_REG
}

void LSM6DS3_ReadAcc(LSM6DS3_IMUData* pfData)
{
	int8_t tmpreg[2] = {0};
	int8_t tmpbuffer11[7] ={0};
	//SPI_ReceiveBytes((uint8_t*)pfData,OUTZ_H_G_REG,7);
	SPI_ReceiveBytes(tmpbuffer11,OUTX_L_XL_REG,6);
	bufferConcat((int8_t*)pfData,tmpbuffer11,6);
	pfData->Data_X += 10;
	pfData->Data_Y += 250;
	pfData->Data_Z -= 200;
	SPI_ReceiveBytes(tmpreg,CTRL1_XL_REG,1);

	/* Switch the sensitivity value set in the CRTL1 */
	switch(tmpreg[1] & 0x0C)
	{
	case 0x00:
		pfData->Senstivity = (float)1/SENSIVITY_XL_2g;
		break;

	case 0x04:
		pfData->Senstivity = (float)1/SENSIVITY_XL_16g;
		break;

	case 0x08:
		pfData->Senstivity = (float)1/SENSIVITY_XL_4g;
		break;

	case 0x0C:
		pfData->Senstivity = (float)1/SENSIVITY_XL_8g;
		break;
	}

}


void LSM6DS3_ReadAngularRate(LSM6DS3_IMUData* pfData)
{
	int8_t tmpreg[2] = {0};
	int8_t tmpbuffer11[7] ={0};
	//SPI_ReceiveBytes((int8_t*)pfData,OUT_TEMP_H_REG,9);

	SPI_ReceiveBytes(tmpbuffer11,OUTX_L_G_REG,6);
	bufferConcat((int8_t*)pfData,tmpbuffer11,6);
	pfData->Data_X -= 25;
	//pfData->Data_Y += 250;
	pfData->Data_Z += 20;
	SPI_ReceiveBytes(tmpreg,CTRL2_G_REG,1);
	/* Switch the sensitivity value set in the CRTL1 */
	switch(tmpreg[1] & 0x0C)
	{
	case 0x00:
		pfData->Senstivity = (float)1/SENSIVITY_G_250dps;
		break;

	case 0x04:
		pfData->Senstivity = (float)1/SENSIVITY_G_500dps;
		break;

	case 0x08:
		pfData->Senstivity = (float)1/SENSIVITY_G_1000dps;
		break;

	case 0x0C:
		pfData->Senstivity = (float)1/SENSIVITY_G_2000dps;
		break;

	default:
		pfData->Senstivity = (float)1/SENSIVITY_G_125dps;
		break;
	}

}

void getInclinationAngleY(float* outData)
{
	LSM6DS3_IMUData gyroData;
	LSM6DS3_IMUData xlData;
	//float Accangle;
	float numirator,denomirator,gyroangle;

	LSM6DS3_ReadAngularRate(&gyroData);
	LSM6DS3_ReadAcc(&xlData);

	numirator = (float)xlData.Data_Y*xlData.Senstivity;
	denomirator = (float)xlData.Data_Z*xlData.Senstivity;
	gyroangle=  (float)gyroData.Data_Y*gyroData.Senstivity;

	Accangle = atan(numirator/denomirator)*57.29;
	*outData = getAngle(Accangle,gyroangle,Dt);
}



float getAngle(float newAngle, float newRate, float dt)
{
	float S;
	float B1;
	float B2;
	float y;
	float P00_temp;
	float P01_temp;
	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	rate = newRate - bias;
	angle += dt * rate;

	// Update estimation error covariance - Project the error covariance ahead
	A1 += dt * (dt*A4 - A2 - A3 + Q_angle);
	A2 -= dt * A4;
	A3 -= dt * A4;
	A4 += Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	S = A1 + R_measure; // Estimate error

	// Kalman gain - This is a 2x1 vector
	B1 = A1 / S;
	B2 = A3 / S;

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	y = newAngle - angle; // Angle difference
	angle += B1 * y;
	bias += B2 * y;

	// Calculate estimation error covariance - Update the error covariance
	P00_temp = A1;
	P01_temp = A2;

	A1 -= B1 * P00_temp;
	A2 -= B1 * P01_temp;
	A3 -= B2 * P00_temp;
	A4 -= B2 * P01_temp;

	return angle;
}

void bufferConcat(int8_t* buf1,int8_t* buf2,int16_t ln)
{
	int i=0;
	while(ln > 0)
	{
		buf1[i] = buf2[i+1];
		i++;
		ln--;
	}
}
