/*
 * HEAVECONTROL.h
 *
 *  Created on: 11 Aug 2016
 *      Author: Arnold
 */

#ifndef HEAVECONTROL_H_
#define HEAVECONTROL_H_

#include "RADIO.h"
#include "PWM.h"

#define HEAVECONTROL_THRUST_MAX 15.0 //[N]
#define HEAVECONTROL_THRUST_MIN 3.0  //[N]

class HEAVECONTROL
{
	public:
		void initialise();
		void stateControl();

		static bool isSaturated;
		static float u;
};


#endif /* HEAVECONTROL_H_ */
