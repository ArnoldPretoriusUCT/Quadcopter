/*
 * IMU.cpp
 *
 *  Created on: 06 Jan 2016
 *      Author: Arnold
 */

#include "IMU.h"

MPU6050 mpu6050 = MPU6050();
Metro calibrationTimer = Metro(IMU_CALIBRATION_TIMER_PERIOD_MSEC);

float IMU::accel[3],
      IMU::accelNotched[3],
	  IMU::gyro[3],
	  IMU::temp,
	  IMU::R_imu2quad[9],
	  IMU::gyroBias[3],
	  IMU::gyroOffset[3];


int16_t IMU::temp_raw,
		IMU::accel_raw[3],
		IMU::gyro_raw[3];

bool IMU::isCalibrating;



void IMU::initialise()
{
	isCalibrating = true;
	memset(&accel[0],0,3*4);
	memset(&gyro[0],0,3*4);
	temp = temp_raw = 0;

	Wire.begin();
	CORE_PIN18_CONFIG = PORT_PCR_MUX(2)|PORT_PCR_ODE|PORT_PCR_SRE|PORT_PCR_DSE;
	CORE_PIN19_CONFIG = PORT_PCR_MUX(2)|PORT_PCR_ODE|PORT_PCR_SRE|PORT_PCR_DSE;
	mpu6050.setSleepEnabled(false);
	mpu6050.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
	mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
	mpu6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

//	mpu6050.setDLPFMode(MPU6050_DLPF_BW_256);
	mpu6050.setDLPFMode(MPU6050_DLPF_BW_98);
//	mpu6050.setDLPFMode(MPU6050_DLPF_BW_42);
//	mpu6050.setDLPFMode(MPU6050_DLPF_BW_20);
//	mpu6050.setDLPFMode(MPU6050_DLPF_BW_5);
}

void IMU::getInitialConditions()
{
	uint32_t cnt = 0;
	float cntMax = 1000;

	float gyroOffsetSum[] = {0,0,0};
	float accelSum[] = {0,0,0};
	float aNom[] = {0,0,1};
	float accel_temp[3],gyro_temp[3];
	float accelInit[3],quatInit[4];

	calibrationTimer.interval(5); //200Hz

	while( isCalibrating == true )
	{
		if(calibrationTimer.check() == true)
		{
			mpu6050.getMotion6(&accel_raw[0],&accel_raw[1],&accel_raw[2],&gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
			temp = (float)mpu6050.getTemperature()/340 + 35;
			float a[3],aCalibrated[3];
			if( cnt < cntMax )
			{
				gyroOffsetSum[0] += (float)gyro_raw[0]/IMU_GYRO_SCALE_FACTOR - IMU_GYRO_GRADIENT_X*temp;
				gyroOffsetSum[1] += (float)gyro_raw[1]/IMU_GYRO_SCALE_FACTOR - IMU_GYRO_GRADIENT_Y*temp;
				gyroOffsetSum[2] += (float)gyro_raw[2]/IMU_GYRO_SCALE_FACTOR - IMU_GYRO_GRADIENT_Z*temp;

				accel_temp[0] = -(float)accel_raw[0]/IMU_ACCEL_SCALE_FACTOR;
				accel_temp[1] =  (float)accel_raw[1]/IMU_ACCEL_SCALE_FACTOR;
				accel_temp[2] = -(float)accel_raw[2]/IMU_ACCEL_SCALE_FACTOR;
				accel[0] = IMU_ACCEL_M11*accel_temp[0] + IMU_ACCEL_M12*accel_temp[1] + IMU_ACCEL_M13*accel_temp[2] + IMU_ACCEL_B1;
				accel[1] = IMU_ACCEL_M21*accel_temp[0] + IMU_ACCEL_M22*accel_temp[1] + IMU_ACCEL_M23*accel_temp[2] + IMU_ACCEL_B2;
				accel[2] = IMU_ACCEL_M31*accel_temp[0] + IMU_ACCEL_M32*accel_temp[1] + IMU_ACCEL_M33*accel_temp[2] + IMU_ACCEL_B3;

				accelSum[0] += accel[0];
				accelSum[1] += accel[1];
				accelSum[2] += accel[2];

				cnt++;
			}
			else
			{
				gyroOffset[0] = gyroOffsetSum[0]/cntMax;
				gyroOffset[1] = gyroOffsetSum[1]/cntMax;
				gyroOffset[2] = gyroOffsetSum[2]/cntMax;

				accelInit[0] = accelSum[0]/cntMax;
				accelInit[1] = accelSum[1]/cntMax;
				accelInit[2] = accelSum[2]/cntMax;

//				aNorm = sqrt( pow(accelInit[0],2) + pow(accelInit[1],2) + pow(accelInit[2],2) );
///				accelInitNormed[0] = accelInit[0]/aNorm;
//				accelInitNormed[1] = accelInit[1]/aNorm;
//				accelInitNormed[2] = accelInit[2]/aNorm;

				accel_temp[0] = (accelInit[0]-accelInit[1])/sqrtf(2);
				accel_temp[1] = (accelInit[0]+accelInit[1])/sqrtf(2);
				accelInit[0] = -accel_temp[0];
				accelInit[1] = -accel_temp[1];

				float aNorm = sqrtf( powf(accelInit[0],2) + powf(accelInit[1],2) + powf(accelInit[2],2) );
				accelInit[0] = accelInit[0]/aNorm;
				accelInit[1] = accelInit[1]/aNorm;
				accelInit[2] = accelInit[2]/aNorm;

				QuaternionMath::getQuaternionFromVectors(accelInit,aNom,&quatInit[0]);
				QuaternionMath::getRotationMatrixFromQuaternion(quatInit,&R_imu2quad[0]);
				isCalibrating = false;
//				mpu6050.setDLPFMode(MPU6050_DLPF_BW_42);
				delay(200);
			}
		}
	}
}

void IMU::getReadings()
{
	float gyro_temp[3],accel_temp[3];;

	mpu6050.getMotion6(&accel_raw[0], &accel_raw[1], &accel_raw[2], &gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
	temp_raw = mpu6050.getTemperature();
	temp = (float)temp_raw/340.0 + 35;
	gyroBias[0] = IMU_GYRO_GRADIENT_X * temp + gyroOffset[0];
	gyroBias[1] = IMU_GYRO_GRADIENT_Y * temp + gyroOffset[1];
	gyroBias[2] = IMU_GYRO_GRADIENT_Z * temp + gyroOffset[2];

//	gyroBias[0]=gyroBias[1]=gyroBias[2]=0;
	gyro[0] = ( (float)gyro_raw[0] / IMU_GYRO_SCALE_FACTOR - gyroBias[0] ) * M_PI / 180;
	gyro[1] = ( (float)gyro_raw[1] / IMU_GYRO_SCALE_FACTOR - gyroBias[1] ) * M_PI / 180;
	gyro[2] = ( (float)gyro_raw[2] / IMU_GYRO_SCALE_FACTOR - gyroBias[2] ) * M_PI / 180;

	accel_temp[0] = (float)accel_raw[0]/IMU_ACCEL_SCALE_FACTOR;
	accel_temp[1] = (float)accel_raw[1]/IMU_ACCEL_SCALE_FACTOR;
	accel_temp[2] = (float)accel_raw[2]/IMU_ACCEL_SCALE_FACTOR;

//	accel[0] = accel_temp[0];
//	accel[1] = accel_temp[1];
//	accel[2] = accel_temp[2];
	accel[0] = IMU_ACCEL_M11*accel_temp[0] + IMU_ACCEL_M12*accel_temp[1] + IMU_ACCEL_M13*accel_temp[2] + IMU_ACCEL_B1;
	accel[1] = IMU_ACCEL_M21*accel_temp[0] + IMU_ACCEL_M22*accel_temp[1] + IMU_ACCEL_M23*accel_temp[2] + IMU_ACCEL_B2;
	accel[2] = IMU_ACCEL_M31*accel_temp[0] + IMU_ACCEL_M32*accel_temp[1] + IMU_ACCEL_M33*accel_temp[2] + IMU_ACCEL_B3;

	static float accel_ux[6],accel_yx[6];
	accel[0] = LPF2(accel[0], dT, 2*M_PI*94, &accel_ux[0], &accel_yx[0]);
	accel[1] = LPF2(accel[1], dT, 2*M_PI*94, &accel_ux[2], &accel_yx[2]);
	accel[2] = LPF2(accel[2], dT, 2*M_PI*94, &accel_ux[4], &accel_yx[4]);

	static float gyro_ux[6],gyro_yx[6];
	gyro[0] = LPF2(gyro[0], dT, 2*M_PI*98, &gyro_ux[0], &gyro_yx[0]);
	gyro[1] = LPF2(gyro[1], dT, 2*M_PI*98, &gyro_ux[2], &gyro_yx[2]);
	gyro[2] = LPF2(gyro[2], dT, 2*M_PI*98, &gyro_ux[4], &gyro_yx[4]);

}

