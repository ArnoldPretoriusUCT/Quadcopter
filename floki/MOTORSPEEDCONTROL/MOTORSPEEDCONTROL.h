/*
 * MOTORSPEEDCONTROL.h
 *
 *  Created on: 11 Aug 2016
 *      Author: Arnold
 */

#ifndef MOTORSPEEDCONTROL_H_
#define MOTORSPEEDCONTROL_H_

#include "Arduino.h"
#include "ENCODER.h"
#include "RADIO.h"
#include "HEAVECONTROL.h"
#include "RATECONTROL.h"

#define MOTORSPEEDCONTROL_BATTERY_VOLTAGE_PIN	 A0
#define MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_UP   0.35
#define MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN 8.0
#define MOTORSPEEDCONTROL_K_INIT				 6.4
#define MOTORSPEEDCONTROL_K_NOM				     	12.77
#define MOTORSPEEDCONTROL_K_DELTA				 0.05
#define MOTORSPEEDCONTROL_W_ZERO		 		 4
#define MOTORSPEEDCONTROL_W_POLE		 		 21
#define MOTORSPEEDCONTROL_W_DELTA				 300.0
#define MOTORSPEEDCONTROL_PWM_MAX 		 	 	 26214.0
#define MOTORSPEEDCONTROL_PWM_MIN 		 	 	 13107.0
#define MOTORSPEEDCONTROL_PWM_SATURATION 		 26000.0
#define MOTORSPEEDCONTROL_FEEDFORWARD_GAIN		 0.4359
#define MOTORSPEEDCONTROL_W0					 600.0

class MOTORSPEEDCONTROL
{
	public:
		static void initialise();
		static void iterate();
		static void getBatteryVoltage();
		static void getMotorCurrents();

		static bool isClosedLoop, isSaturated, torqueFeedForward, dwEnabled;
		static float reference;
		static float K[4],w,dw,batteryVoltage;

	private:
		static float r,b[4],u_sat[4],u_blade[4];

};


#endif /* MOTORSPEEDCONTROL_H_ */
