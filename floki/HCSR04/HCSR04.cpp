/*
 * HCSR04.cpp
 *
 *  Created on: 30 Nov 2015
 *      Author: Arnold
 */
#include "HCSR04.h"
uint16_t HCSR04::echoCount;
int16_t HCSR04::dtRisingEdge;
bool HCSR04::ultraSonicTimeout;
float HCSR04::distance;
IntervalTimer HCSR04::triggerTimer,
			  HCSR04::preLoopTimer;


HCSR04::HCSR04()
{
	echoCount = 0;
	ultraSonicTimeout = false;
	distance = .03;
	dtRisingEdge = 0;

	pinMode(HCSR04_TRIGGER_PIN,OUTPUT);
	pinMode(HCSR04_ECHO_PIN,INPUT);
}

void HCSR04::initialise()
{
	// Input filter to help prevent glitches from triggering the capture
	// 4+4×val clock cycles, 48MHz = 4+4*7 = 32 clock cycles = 0.75us
//	FTM0_FILTER = 0xF;
	// Must set the Write-protect disable (WPDIS) bit to allow modifying other registers
	// The enable (FTMEN) bit is also set to enable the FlexTimer0 module
	// FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=1
	FTM0_MODE  ^= 0b100;
//	FTM0_SC = 0x00; // Set this to zero before changing the modulus
//	FTM0_CNT = 0x00; // Reset the count to zero
//	FTM0_MOD = 0xFFFF; // max modulus = 65535
//	FTM0_SC |= 0b1000000;; // TOF=0 TOIE=1 CPWMS=0 CLKS=01 (system clock) PS=111 (divide by 128)
	FTM0_C2SC = 0x4C; //dual edge capture

	// Enable FTM0 interrupt inside NVIC
	NVIC_ENABLE_IRQ(IRQ_FTM0);
	PORTC_PCR3 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;  //PIN 9(46) ALT 4
}

extern "C" void ftm0_isr(void)
{
	static bool t0_read,t1_read;
	static uint16_t t1,t0,t1_past,t_wait = 0;
	static unsigned char overflowCount = 0;

	if( (FTM0_C2SC & 0x80) != 0 )
	{
		if( digitalRead(HCSR04_ECHO_PIN) == HIGH )
		{
			t0 = (uint16_t)FTM0_C2V;
			t0_read = true;
		}
		else if( digitalRead(HCSR04_ECHO_PIN) == LOW )
		{
			t1 = (uint16_t)FTM0_C2V;
			t1_read = true;
		}
		if ( t0_read == true && t1_read == true )
		{
			HCSR04::echoCount = (uint16_t)t1 - (int16_t)t0;
			HCSR04::dtRisingEdge = t1 - t1_past;
			t1_past = (int16_t)t1;

			t0_read = t1_read = false;
			t_wait = 0;
			overflowCount = 0;
			HCSR04::ultraSonicTimeout = false;

			uint16_t preLoopWait = 1.0e6*( dT - HCSR04::getEchoTime()) - 20; //microseconds
//			HCSR04::preLoopTimer.begin(HCSR04::startTrigger,preLoopWait);
//			Serial.println(preLoopWait);
		}
		FTM0_C2SC &= ~0x80;
	}
	else
	{
		t_wait += 65535 - t0;
		if( t_wait >= 65535 )
		{
			HCSR04::ultraSonicTimeout = true;
			HCSR04::startTrigger(); //TODO:make another timer to try reestablish constant churp reception
			t_wait = 0;
		}
//		FTM2_SC &= ~0x80;
	}
}

void HCSR04::stopTrigger()
{
  digitalWrite(HCSR04_TRIGGER_PIN, LOW);
  HCSR04::triggerTimer.end();
}

void HCSR04::startTrigger()
{
	digitalWrite(HCSR04_TRIGGER_PIN,HIGH);
	HCSR04::triggerTimer.begin(HCSR04::stopTrigger,10);
}

float HCSR04::getEchoTime()
{
	float echoTime = (float)HCSR04::echoCount/HCSR04_TIMER_FREQ;
	return echoTime;
}

float HCSR04::getEchoFrequency()
{
	float freq = HCSR04_TIMER_FREQ/(float)dtRisingEdge;
	if( dtRisingEdge == 0)
	{
		freq = 0;
	}
	return freq;
}

void HCSR04::getDistance()
{
	distance = (float)echoCount/HCSR04_TIMER_FREQ * 340.0/2;
}

bool HCSR04::isTimedOut()
{
	return ultraSonicTimeout;
}
