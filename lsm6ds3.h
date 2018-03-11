/*
 * lsm6ds1.h
 *
 *  Created on: Dec 7, 2017
 *      Author:  Umar Yaqoob
 */

#ifndef BSP_HAL_INC_LSM6DS3_H_
#define BSP_HAL_INC_LSM6DS3_H_
/*
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "stdint.h"
#include "stm32f10x.h"
*/
#include "stdint.h"
#include "stm32f10x.h"
/* LSM6DS3 struct */
typedef struct
{
	uint8_t Output_DataRate;                    /* OUT data rate */
	uint8_t BlockData_Update;                   /* Block Data Update */
	uint8_t Endianness;                         /* Endian Data selection */
	uint8_t Full_Scale;                         /* Full Scale selection */
	uint8_t Register_read;
}LSM6DS3Gyro_InitTypeDef;

typedef struct
{
	uint8_t Output_DataRate;                    /* OUT data rate */
	uint8_t Band_Width;                         /* Bandwidth selection */
	uint8_t BandWidth_Source;                   /* Either user defined value or depends on ODR */
	uint8_t BlockData_Update;                   /* Block Data Update */
	uint8_t Endianness;                         /* Endian Data selection */
	uint8_t Full_Scale;                         /* Full Scale selection */
	uint8_t Register_read;
}LSM6DS3XL_InitTypeDef;


typedef struct
{
//	int16_t junk;
	int16_t Data_X;
	int16_t Data_Y;
	int16_t Data_Z;
	float Senstivity;
}LSM6DS3_IMUData;

typedef struct
{
	double  roll;
	double  pitch;
	double  yaw;
}LSM6DS3_EulerAngles;

typedef struct
{
	int8_t Data_0;
	int8_t Data_1;
	int8_t Data_2;
	int8_t Data_3;
	int8_t Data_4;
	int8_t Data_5;
	int8_t Data_6;
}RawData;


#define READ_CMD                    (int8_t)0x80
#define DUMMY_BYTE                  (int8_t)0x00

/*****************************************/
/******** LSM6DS3 REGISTER MAPING ********/
/*****************************************/

#define FUNC_CFG_ACCESS_REG		    (uint8_t)0x01
#define SENSOR_SYNC_TIME_FRAME_REG	(uint8_t)0x04
#define FIFO_CTRL1_REG         		(uint8_t)0x06
#define FIFO_CTRL2_REG		        (uint8_t)0x07
#define FIFO_CTRL3_REG				(uint8_t)0x08
#define FIFO_CTRL4_REG				(uint8_t)0x09
#define FIFO_CTRL5_REG				(uint8_t)0x0A
#define ORIENT_CFG_G_REG			(uint8_t)0x0B
#define INT1_CTRL_REG				(uint8_t)0x0D
#define INT2_CTRL_REG				(uint8_t)0x0E
#define WHO_AM_I_REG				(int8_t)0x0F
#define CTRL1_XL_REG				(int8_t)0x10
#define CTRL2_G_REG					(int8_t)0x11
#define CTRL3_C_REG					(int8_t)0x12
#define CTRL4_C_REG					(int8_t)0x13
#define CTRL5_C_REG					(int8_t)0x14
#define CTRL6_C_REG					(int8_t)0x15
#define CTRL7_G_REG					(int8_t)0x16
#define CTRL8_XL_REG				(int8_t)0x17
#define CTRL9_XL_REG				(int8_t)0x18
#define CTRL10_C_REG				(uint8_t)0x19
#define MASTER_CONFIG_REG			(uint8_t)0x1A
#define WAKE_UP_SRC_REG				(uint8_t)0x1B
#define TAP_SRC_REG					(uint8_t)0x1C
#define D6D_SRC_REG					(uint8_t)0x1D
#define STATUS_REG_REG				(uint8_t)0x1E
#define OUT_TEMP_L_REG				(uint8_t)0x20
#define OUT_TEMP_H_REG				(uint8_t)0x21
#define OUTX_L_G_REG				(int8_t)0x22
#define OUTX_H_G_REG				(int8_t)0x23
#define OUTY_L_G_REG				(int8_t)0x24
#define OUTY_H_G_REG				(int8_t)0x25
#define OUTZ_L_G_REG				(int8_t)0x26
#define OUTZ_H_G_REG				(int8_t)0x27
#define OUTX_L_XL_REG				(int8_t)0x28
#define OUTX_H_XL_REG				(int8_t)0x29
#define OUTY_L_XL_REG				(int8_t)0x2A
#define OUTY_H_XL_REG				(int8_t)0x2B
#define OUTZ_L_XL_REG				(int8_t)0x2C
#define OUTZ_H_XL_REG				(int8_t)0x2D
#define SENSORHUB1_REG				(uint8_t)0x2E
#define SENSORHUB2_REG				(uint8_t)0x2F
#define SENSORHUB3_REG	 			(uint8_t)0x30
#define SENSORHUB4_REG				(uint8_t)0x31
#define SENSORHUB5_REG				(uint8_t)0x32
#define SENSORHUB6_REG				(uint8_t)0x33
#define SENSORHUB7_REG				(uint8_t)0x34
#define SENSORHUB8_REG				(uint8_t)0x35
#define SENSORHUB9_REG				(uint8_t)0x36
#define SENSORHUB10_REG				(uint8_t)0x37
#define SENSORHUB11_REG				(uint8_t)0x38
#define SENSORHUB12_REG				(uint8_t)0x39
#define FIFO_STATUS1_REG			(uint8_t)0x3A
#define FIFO_STATUS2_REG			(uint8_t)0x3B
#define FIFO_STATUS3_REG			(uint8_t)0x3C
#define FIFO_STATUS4_REG			(uint8_t)0x3D
#define FIFO_DATA_OUT_L_REG			(uint8_t)0x3E
#define FIFO_DATA_OUT_H_REG			(uint8_t)0x3F
#define TIMESTAMP0_REG				(uint8_t)0x40
#define TIMESTAMP1_REG				(uint8_t)0x41
#define TIMESTAMP2_REG				(uint8_t)0x42
#define STEP_TIMESTAMP_L_REG		(uint8_t)0x49
#define STEP_TIMESTAMP_H_REG		(uint8_t)0x4A
#define STEP_COUNTER_L_REG			(uint8_t)0x4B
#define STEP_COUNTER_H_REG			(uint8_t)0x4C
#define SENSORHUB13_REG				(uint8_t)0x4D
#define SENSORHUB14_REG				(uint8_t)0x4E
#define SENSORHUB15_REG				(uint8_t)0x4F
#define SENSORHUB16_REG				(uint8_t)0x50
#define SENSORHUB17_REG				(uint8_t)0x51
#define SENSORHUB18_REG				(uint8_t)0x52
#define FUNC_SRC_REG				(uint8_t)0x53
#define TAP_CFG_REG					(uint8_t)0x58
#define TAP_THS_6D_REG				(uint8_t)0x59
#define INT_DUR2_REG				(uint8_t)0x5A
#define WAKE_UP_THS_REG				(uint8_t)0x5B
#define WAKE_UP_DUR_REG				(uint8_t)0x5C
#define FREE_FALL_REG				(uint8_t)0x5D
#define MD1_CFG_REG					(uint8_t)0x5E
#define MD2_CFG_REG					(uint8_t)0x5F
#define OUT_MAG_RAW_X_L_REG			(uint8_t)0x66
#define OUT_MAG_RAW_X_H_REG			(uint8_t)0x67
#define OUT_MAG_RAW_Y_L_REG			(uint8_t)0x68
#define OUT_MAG_RAW_Y_H_REG			(uint8_t)0x69
#define OUT_MAG_RAW_Z_L_REG			(uint8_t)0x6A
#define OUT_MAG_RAW_Z_H_REG			(uint8_t)0x6B

/*************************************/
/******** REGISTER MAPING END ********/
/*************************************/

/* @DEF accelerometer ODR selection */
#define ODR_XL_PD                   (uint8_t)0x00
#define ODR_XL_12Hz                 (uint8_t)0x10
#define ODR_XL_26Hz                 (uint8_t)0x20
#define ODR_XL_52Hz                 (uint8_t)0x30
#define ODR_XL_104Hz                (uint8_t)0x40
#define ODR_XL_208Hz                (uint8_t)0x50
#define ODR_XL_416Hz                (uint8_t)0x60
#define ODR_XL_833Hz                (uint8_t)0x70
#define ODR_XL_1k66Hz               (uint8_t)0x80
#define ODR_XL_3k33Hz               (uint8_t)0x90
#define ODR_XL_6k66Hz               (uint8_t)0xA0
/* @END_DEF accelerometer ODR selection */

/* @DEF Gyroscope ODR selection */
#define ODR_G_PD                    ((uint8_t)0x00)
#define ODR_G_12Hz                  ((uint8_t)0x10)
#define ODR_G_26Hz                  ((uint8_t)0x20)
#define ODR_G_52Hz                  ((uint8_t)0x30)
#define ODR_G_104Hz                 ((uint8_t)0x40)
#define ODR_G_208Hz                 ((uint8_t)0x50)
#define ODR_G_416Hz                 ((uint8_t)0x60)
#define ODR_G_833Hz                 ((uint8_t)0x70)
#define ODR_G_1k66Hz                ((uint8_t)0x80)
/* @END_DEF Gyroscope ODR selection */


/* @DEF accelerometer FULL SCALE selection */
#define FS_XL_2g                    (uint8_t)0x00
#define FS_XL_4g                    (uint8_t)0x08
#define FS_XL_8g                    (uint8_t)0x0C
#define FS_XL_16g                   (uint8_t)0x04
/* @END_DEF accelerometer FULL SCALE selection */

/* @DEF Gyroscope FULL SCALE selection */
#define FS_G_250dps                 (uint8_t)0x00
#define FS_G_500dps                 (uint8_t)0x04
#define FS_G_1000dps                (uint8_t)0x08
#define FS_G_2000dps                (uint8_t)0x0C
/* @END_DEF Gyroscope FULL SCALE selection */

/* @DEF Accelerometer BANDWIDTH selection */
#define BW_XL_50Hz                  (uint8_t)0x03
#define BW_XL_100Hz                 (uint8_t)0x02
#define BW_XL_200Hz                 (uint8_t)0x01
#define BW_XL_400Hz                 (uint8_t)0x00
/* @END_DEF Accelerometer BANDWIDTH selection */

/* @DEF Accelerometer BANDWIDTH selection source */
#define BandWidth_UserDefined      ((uint8_t)0x80)
#define BandWidth_Default          ((uint8_t)0x00)
/* @END_DEF Accelerometer BANDWIDTH selection source */


/* @DEF Block data update */
#define BlockDataUpdate_Continous   ((uint8_t)0x00)
#define BlockDataUpdate_Single      ((uint8_t)0x40)
/* @END_DEF Block data update */

/* @DEF Endian_Data_selection */
#define LSB_FIRST                   ((uint8_t)0x02)
#define MSB_FIRST	                ((uint8_t)0x00)
/* @END_DEF Endian_Data_selection */

/* @DEF Register read mode */
#define Register_read_continuous    ((uint8_t)0x04)
#define Register_read_still	        ((uint8_t)0x00)
/* @END_DEF Register read mode */

void LSM6DS3_init(void);
void LSM6DS3_ReadAngularRate(LSM6DS3_IMUData* pfData);
void LSM6DS3_ReadAcc(LSM6DS3_IMUData* pfData);
void getInclinationAngleY(float* outData);
void readRawValues(int8_t* data);
void who_am_i(void);


#endif /* BSP_HAL_INC_LSM6DS1_H_ */
