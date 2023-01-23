/*
 * WATCHDOG.cpp
 *
 *  Created on: 11 Aug 2016
 *      Author: Arnold
 */

#include "WATCHDOG.h"

uint16_t WATCHDOG::dogTime;
bool WATCHDOG::resetMicro,
	 WATCHDOG::isCritical;

void WATCHDOG::initialise()
{
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
	WDOG_TOVALL = 500; // The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to.
	WDOG_TOVALH = 0;
	WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG

	WDOG_PRESC = 0; // This sets prescale clock so that the watchdog timer ticks at 1kHZ instead of the default 1kHZ/4 = 200 HZ}

	isCritical = false;
}

void WATCHDOG::refresh()
{
//	uint16_t currentTime = millis();
//	if( (currentTime - dogTime) > 1 ) //fix
//	{
//		noInterrupts();
//		WDOG_REFRESH = 0xA602;
//		WDOG_REFRESH = 0xB480;
//		interrupts();
//
//		dogTime = millis();
//	}
	if( resetMicro == true )
	{
		SCB_AIRCR =  (0x5FA << 16) | 0b100; //reset micro-controller
		delay(2000);
	}
	if( isCritical )
	{
		digitalWrite(LED_STATUS_PIN,true);
	}
}


