/*
 * mpu_imu_6050.c
 *
 *  Created on: Dec 6, 2018
 *      Author: user
 */


#include "mpu_imu_6050.h"

//-------defintion and implementation of functions---------//

//initialization of MPU module by waking up from sleep mode and configuration of scale.
void MPU6050_Init(I2C_HandleTypeDef *hi2c,mpu6050_I2C_Address address, mpu6050_Gyro_FS gyro_FS,mpu6050_Accel_FS accel_FS,mpu6050_Sampling_Rate sampling_rate)
{

	uint8_t data_buffer[2];
	//-----sleepMode---------//
	/*to be done*/

	//-----set full scale of gyroscope------//
	data_buffer[0]= MPU6050_GYRO_CONFIG;
	data_buffer[1]=gyro_FS;
	HAL_I2C_Master_Transmit(hi2c,address,data_buffer,2,);//2 means you want to transimt two data_phases the address of register you want to write in and the data you want to write

	//------set full scale of accelerometer---------//
	data_buffer[0]= MPU6050_ACCEL_CONFIG;
	data_buffer[1]=accel_FS;
	HAL_I2C_Master_Transmit(hi2c,address,data_buffer,2,);

	//------set sampling rate-----//
	data_buffer[0]=MPU6050_SMPLRT_DIV;
	data_buffer[1]=sampling_rate;
	HAL_I2C_Master_Transmit(hi2c,address,data_buffer,2,);
}
//2 questions here will be asked in meeting casting of address as it's 0xD2 8 bits but in hal fn it's 16 bits and time_out in HAL_I2C_Master_Transmit and handler

//function to read data from registers and stores them in struct

void MPU6050_Read_Data_fromReg(I2C_HandleTypeDef *hi2c,mpu6050_I2C_Address address,mpu6050_Readings* mpu_pointer_to_theStruct)
{
	uint8_t data_buffer[7]={0};//7 bec 1 for reg address and 6 for readings(1 for xhigh 1 for xlow the yhigh ylow zhigh zlow)
	//------reading of gyroscope----//
	data_buffer[0]=MPU6050_GYRO_XOUT_H;
	HAL_I2C_Master_Transmit(hi2c,address,data_buffer,1,);
	HAL_I2C_Master_Receive(hi2c,address,&data_buffer[1],6,);// at this point 6 data phases transmitted from registers to data_buffer starting at index 1 and ends at index 6(index 1 for xhigh 2 for xlow 3 for yhigh 4 for ylow 5 for zhigh 6 for zlow)

	//now construct readings so shift high and and add low (oring with low)
	mpu_pointer_to_theStruct->gyro_x=(int16_t)(data_buffer[1]<<8)| data_buffer[2];
	mpu_pointer_to_theStruct->gyro_y=(int16_t)(data_buffer[3]<<8)| data_buffer[4];
	mpu_pointer_to_theStruct->gyro_z=(int16_t)(data_buffer[5]<<8)| data_buffer[6];
/*needed to add delay*/

	//----reading of accelerometer----//
	data_buffer[0]=MPU6050_ACCEL_XOUT_H;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,1,);
	HAL_I2C_Master_Receive(hi2c,(uint16_t)address,&data_buffer[1],6,);
	mpu_pointer_to_theStruct->acc_x=(int16_t)(data_buffer[1]<<8)| data_buffer[2];
	mpu_pointer_to_theStruct->acc_y=(int16_t)(data_buffer[3]<<8)| data_buffer[4];
	mpu_pointer_to_theStruct->acc_z=(int16_t)(data_buffer[5]<<8)| data_buffer[6];
/*needed to add delay*/
}
