/*
 * HCSR04.h
 *
 *  Created on: 30 Nov 2015
 *      Author: Arnold
 */

#ifndef HCSR04_H_
#define HCSR04_H_

//#include "../arduino/core/IntervalTimer.h"
#include "Arduino.h"
#include <stdbool.h>
#include "IntervalTimer.h"
#include "common.h"

#define HCSR04_TRIGGER_PIN 14 //changed from 12
#define HCSR04_ECHO_PIN    9
#define HCSR04_TIMER_FREQ  12e6 // 48MHz/4

class HCSR04
{
	public:
		HCSR04();
		void initialise();
		static void stopTrigger();
		static void startTrigger();

		static float getEchoTime();
		float getEchoFrequency();
		void getDistance();

		bool isTimedOut();

		static uint16_t echoCount;
		static int16_t dtRisingEdge;
		static bool ultraSonicTimeout;
		static float distance;
		static IntervalTimer preLoopTimer;
	private:
		static IntervalTimer triggerTimer;

};

#endif /* HC-SR04_H_ */
