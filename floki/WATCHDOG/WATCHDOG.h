/*
 * WATCHDOG.h
 *
 *  Created on: 11 Aug 2016
 *      Author: Arnold
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "Arduino.h"
#include "common.h"
#include "LED.h"

class WATCHDOG
{
	public:
		static void initialise();
		static void refresh();

		static bool resetMicro,isCritical;

	private:
		static uint16_t dogTime;
};


#endif /* WATCHDOG_H_ */
