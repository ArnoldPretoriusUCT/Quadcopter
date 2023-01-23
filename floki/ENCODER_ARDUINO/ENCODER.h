/*
 * ENCODER.h
 *
 *  Created on: 03 Dec 2015
 *      Author: Arnold
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "Arduino.h"
#include <stdbool.h>
#include "common.h"
#include "MOTORSPEEDCONTROL.h"
#include "PWM.h"
#include "WATCHDOG.h"

#define ENCODER_PSC 				128.0
#define ENCODER_CPU_FRQ 			48000000.0
#define ENCODER_WATCHDOG_THRESHOLD	100
#define ENCODER_ESC_THRESHOLD 		20000

class ENCODER
{
	public:
		static void initialise();
		static void getMotorSpeed();
		static void watchDog();

		static uint32_t FTM1_Ch0Count,FTM1_Ch1Count,FTM2_Ch0Count,FTM2_Ch1Count;
		static uint16_t FTM1_Ch0Count_past,FTM1_Ch1Count_past,FTM2_Ch0Count_past,FTM2_Ch1Count_past;
		static float motorSpeed[4];

		static uint32_t FTM1_tickCount[2],FTM2_tickCount[2];
		static uint8_t FTM1_overflowCount[2],FTM2_overflowCount[2];
		static bool FTM1_isValid[2],FTM2_isValid[2];
	private:
		static uint8_t watchDogCount;
};



#endif /* ENCODER_H_ */
