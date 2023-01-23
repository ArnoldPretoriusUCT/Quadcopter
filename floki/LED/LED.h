/*
 * LED.h
 *
 *  Created on: 9 Feb 2017
 *      Author: Arnold
 */

#include "Arduino.h"

#ifndef LED_H_
#define LED_H_

#define LED_IR_PIN 12
#define LED_STATUS_PIN 13
#define LED_FLAG1_PIN 11
#define LED_FLAG2_PIN 16
#define LED_FLAG3_PIN 17

class LED
{
	public:
		static void initialise();
		static void controlIR();
		static void toggleStatusLED();

		static bool isEnabled;

	private:
		static bool toggleState;
};


#endif /* LED_H_ */
