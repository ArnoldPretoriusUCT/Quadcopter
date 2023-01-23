/*
 * PWM.h
 *
 *  Created on: 03 Dec 2015
 *      Author: Arnold
 */

#ifndef PWM_H_
#define PWM_H_

#include "Arduino.h"
#include "HEAVECONTROL.h"
#include "RATECONTROL.h"
#include "MOTORSPEEDCONTROL.h"
#include "RADIO.h"
#include "WIXEL.h"
#include "THRUSTCONTROL.h"

#define PWM_SERVO1 		  		23
#define PWM_SERVO2 		  		20
#define PWM_SERVO3 		  		5
#define PWM_SERVO4 		  		6

#define PWM_ESC1  		  		9
#define PWM_ESC2  		  		10
#define PWM_ESC3				22
#define PWM_ESC4				21

#define PWM_ANALOG_RES 	  		16
#define PWM_FREQUENCY			200
#define PWM_SERVO_DELTA_MAX 	3500
#define PWM_SERVO1_OFFSET 		19661
#define PWM_SERVO2_OFFSET 		19661
#define PWM_SERVO3_OFFSET 		(19661+590-1417-1185+600)
#define PWM_SERVO4_OFFSET 		19661
#define PWM_SERVO_DELTA_INIT	500
#define PWM_ESC_CALIBRATION_COM 16000

class PWM
{
	public:
		static void initialise();
		static void write();
		static void servoMapping();

		static float escCom[4];
		static bool  isCalibrating,motorsArmed,servoCentering,criticalFailure;
		static float servoCom[4],servoComDelta[4],servoOffset[4],servoCalibrationCom;
		static float ledBlobCom;

		static float U[4];
		static bool radioClear;

	private:
		static float servoComLowerLimit[4],servoComUpperLimit[4];
};



#endif /* PWM_H_ */
