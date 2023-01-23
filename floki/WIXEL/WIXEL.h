/*
 * WIXEL.h
 *
 *  Created on: 06 Jun 2016
 *      Author: Arnold
 */

#ifndef WIXEL_H_
#define WIXEL_H_

#include "Arduino.h"
#include "PWM.h"
#include "IMU.h"
#include "ENCODER.h"
#include "MOTORSPEEDCONTROL.h"
#include "HEAVECONTROL.h"
#include "ATTITUDECONTROL.h"
#include "ECF.h"
#include "RADIO.h"
#include "common.h"
#include "LED.h"
#include "WATCHDOG.h"
#include "THRUSTCONTROL.h"


#define WIXEL_TX							Serial3
#define WIXEL_RX							Serial1
#define WIXEL_BAUD_RATE 					200000//350000
#define WIXEL_SEND_BUFFER_SIZE 				27
#define WIXEL_RECEIVE_BUFFER_SIZE 			18
#define WIXEL_RECEIVE_WATCH_DOG_THRESHOLD   20 //50x5=250ms
//
#define WIXEL_VELOCITY_SF 					5000
#define WIXEL_ACCELERATION_SF 				(IMU_ACCEL_SCALE_FACTOR/g)
#define WIXEL_ALTITUDE_SF 					1.6384e4
#define WIXEL_HEAVE_SF 				 		2000
#define WIXEL_QUATERNION_SF					32767
#define WIXEL_GYRO_SF 						(IMU_GYRO_SCALE_FACTOR*180.0/M_PI)
#define WIXEL_ACCEL_SF 						IMU_ACCEL_SCALE_FACTOR
#define WIXEL_ESC_SF						(1.0/51.4)
#define WIXEL_SERVO_SF						127.0/PWM_SERVO_DELTA_MAX
#define WIXEL_MOTOR_SF             			(1.0/3.92)
#define WIXEL_ANGLE_REFERENCE_SF 			(65535/2/M_PI)
#define WIXEL_V_BAT_SF						(255.0/12.4876)
#define WIXEL_CURRENT_SF					25.5

#define WIXEL_ROTRATE_SF					(IMU_GYRO_SCALE_FACTOR*180.0/M_PI)
#define WIXEL_ATTITUDE_CONTROL_KQ_SF		100
#define WIXEL_ATTITUDE_CONTROL_KW_SF		100
#define WIXEL_MOTOR_K_SF					100
#define WIXEL_MOTOR_W_SF					100

class WIXEL
{
	public:
		static void initialise();
		static void getData();
		static void sendData();
		static void getReceiveFrequency();
		static void watchDog();

		static float receiveFrequency,aRead,aRead_buffer[4];
		static bool isOpen, commsState, newData;

	private:
		static uint16_t packetCount;
		static char watchDogCount;
		static unsigned char receiveBuffer[WIXEL_RECEIVE_BUFFER_SIZE];
		static unsigned char sendBuffer[WIXEL_SEND_BUFFER_SIZE];
		static float sendTime[2],receiveTime[2];

		static uint8_t packetID;
};





#endif /* WIXEL_H_ */
