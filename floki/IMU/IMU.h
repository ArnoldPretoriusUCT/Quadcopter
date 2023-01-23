/*
 * IMU.h
 *
 *  Created on: 06 Jan 2016
 *      Author: Arnold
 */

#ifndef IMU_H_
#define IMU_H_

#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MatrixMath.h"
#include "Metro.h"
#include "QuaternionMath.h"

#define IMU_GYRO_SCALE_FACTOR 	32.8
#define IMU_ACCEL_SCALE_FACTOR 	16384.0

#define IMU_GYRO_GRADIENT_X    -0.01469//-0.0366
#define IMU_GYRO_GRADIENT_Y    -0.0027191//0.0
#define IMU_GYRO_GRADIENT_Z     0.013444//0.0442

#define IMU_ACCEL_M11  		    0.992550707302906//0.9933
#define IMU_ACCEL_M12  		   -0.029264504871207//0.0062
#define IMU_ACCEL_M13  			0.073494999169504//0.0032
#define IMU_ACCEL_M21  			0.023969211641661//0.0019
#define IMU_ACCEL_M22  			0.992965131610110//0.9997
#define IMU_ACCEL_M23  		   -0.014277671108785//0.0077
#define IMU_ACCEL_M31 		   -0.080127826763816//-0.0074
#define IMU_ACCEL_M32 		    0.027981327229886//-0.0040
#define IMU_ACCEL_M33 		 	0.956798445983497//0.9762
#define IMU_ACCEL_B1   			0.000318450987918//0.0045
#define IMU_ACCEL_B2   		   -0.014126689977361//0.0010
#define IMU_ACCEL_B3   		   -0.046989155331524//-0.0353

#define IMU_CALIBRATION_TIMER_PERIOD_MSEC 5 //[msec]

class IMU
{
	public:
		static void initialise();
		static void getInitialConditions();
		static void getReadings();

		static float accel[3],gyro[3],temp;
		static float accelNotched[3];
		static int16_t temp_raw;
		static float R_imu2quad[9];

		static bool isCalibrating;

	private:
		static int16_t accel_raw[3],gyro_raw[3];
		static float gyroBias[3],gyroOffset[3];
};



#endif /* IMU_H_ */
