/*
 * HEAVECONTROL.cpp
 *
 *  Created on: 11 Aug 2016
 *      Author: Arnold
 */

#include "HEAVECONTROL.h"

bool HEAVECONTROL::isSaturated;
float HEAVECONTROL::u;

void HEAVECONTROL::initialise()
{
	isSaturated = false;
	u = HEAVECONTROL_THRUST_MIN;
}

void HEAVECONTROL::stateControl()
{
	if( !PWM::isCalibrating && !RADIO::isOpen )
	{
		u = HEAVECONTROL_THRUST_MIN;
	}

	if( u < HEAVECONTROL_THRUST_MIN )
	{
		u = HEAVECONTROL_THRUST_MIN;
	}
}
