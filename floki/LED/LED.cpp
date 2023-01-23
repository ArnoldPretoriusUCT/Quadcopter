/*
 * LED.cpp
 *
 *  Created on: 9 Feb 2017
 *      Author: Arnold
 */
#include "LED.h"

bool LED::isEnabled,
	 LED::toggleState;

void LED::initialise()
{
	pinMode(LED_IR_PIN,1);
	pinMode(LED_STATUS_PIN,1);
	pinMode(LED_FLAG1_PIN,1);
	pinMode(LED_FLAG2_PIN,1);
	pinMode(LED_FLAG3_PIN,1);
	digitalWrite(LED_STATUS_PIN,false);
	digitalWrite(LED_FLAG1_PIN,false);
	digitalWrite(LED_FLAG2_PIN,false);
	digitalWrite(LED_FLAG3_PIN,false);
	LED::isEnabled = false;

	toggleState = false;
}

void LED::controlIR()
{
	if( LED::isEnabled == true )
	{
		digitalWrite(LED_IR_PIN,1);
	}
	else
	{
		digitalWrite(LED_IR_PIN,0);
	}
}

void LED::toggleStatusLED()
{
	digitalWrite(LED_STATUS_PIN,toggleState);
	toggleState = !toggleState;
}



