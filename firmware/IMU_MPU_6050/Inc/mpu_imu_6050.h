/*
 * MPU_6050.h
 *
 *  Created on: Dec 6, 2018
 *      Author: khaled ahmed abdelgalil
 */

#ifndef MPU_6050_H_
#define MPU_6050_H_



//-----------register addresses from register map data_sheet--------------//
//Note:H means it contains the 8 most significant bits, L  contains the 8 least significant bits as reading is 16 bits.
#define MPU6050_SMPLRT_DIV 0x19//25 in decimal-----> equation of simpling_Rate :Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) from data_sheet of register_map
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I   0x75

//enum holds definitions for addresses of I2C devices as if we want to have more than MPU device connected to the same micro_controller
typedef enum
{
	//-----mpu6050 address is 110100x 7bits----//

	MPU6050_ADDRESS_AD0_LOW=0xD0,//when AD0 is grounded the address of I2C device(MPU) is 0x68 this what the data sheet says but I made it 0xD0 as Hal function takes it shifted
	MPU6050_ADDRESS_AD0_HIGH=0xD2//when AD0 is VCC the address of I2C device(MPU) is 0x69 this what the data sheet says but I made it 0xD2 as Hal function takes it shifted

}mpu6050_I2C_Address;

//-----------scales of gryoscope and accelerometer->from data_sheet of register mapping------//
typedef enum
{
	FULLSCALE_250=0,
	FULLSCALE_500=1,
	FULLSCALE_1000=2,
	FULLSCALE_2000=3

}mpu6050_Gyro_FS;

typedef enum
{
	FULLSCALE_2g=0,
	FULLSCALE_4g=1,
	FULLSCALE_8g=2,
	FULLSCALE_16g=3

}mpu6050_Accel_FS;

//----------simpling Rate------------//
typedef enum
{
	//-----sample rate equation from data_sheet of register mapping= gyroscope output rate / ( 1 + SMPLRT_DIV ) ------// gryoscope output rate is 8khz

	//----these values are stored in SMPLRT_DIV register------//
	SAMPLE_RATE_8KHz=0,
	SAMPLE_RATE_4KHz=1,
	SAMPLE_RATE_2KHz=3,
	SAMPLE_RATE_1KHz=7,
	SAMPLE_RATE_500Hz=15,
	SAMPLE_RATE_250Hz=31,
	SAMPLE_RATE_125Hz=63,
	SAMPLE_RATE_100Hz=79

}MPU6050_Sampling_Rate;

//--------data struct to hold readings---------// contains Raw readings
typedef struct
{
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;

}mpu6050_Readings;

#endif /* MPU_6050_H_ */
