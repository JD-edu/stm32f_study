/*
 * MPU9250_2.h
 *
 *  Created on: Aug 11, 2023
 *      Author: kroker
 */

#ifndef INC_MPU9250_2_H_
#define INC_MPU9250_2_H_

#include "stdint.h"
#include "main.h"
#include "i2c.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_MPU9250   0x75 // Should return 0x71
#define PWR_MGMT_1         0x6B // Device defaults to the SLEEP mode
#define SMPLRT_DIV         0x19

#define GYRO_CONFIG        0x1B
#define GYRO_XOUT_H		   0x43
#define GYRO_YOUT_H        0x45
#define GYRO_ZOUT_H		   0x47

#define ACCEL_CONFIG       0x1C
#define ACCEL_XOUT_H       0x3B
#define ACCEL_YOUT_H	   0x3D
#define ACCEL_ZOUT_H	   0x3F

typedef struct{

	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;

	double Ax;
	double Ay;
	double Az;

	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;

	double Gx;
	double Gy;
	double Gz;

	double KalmanAngleX;
	double KalmanAngleY;
	double KalmanAngleZ;

	double sdt;
	int init_flag;
}MPU9250_t;


typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;



uint8_t MPU9250_Init(I2C_HandleTypeDef *I2Cx);
void MPU9250_Read_Acc(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct);
void MPU9250_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct);
void MPU9250_Read_All(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct);
double Kalman_getAngle(Kalman_t *kalman, double newAngle, double newRate, double dt);



#endif /* INC_MPU9250_2_H_ */
