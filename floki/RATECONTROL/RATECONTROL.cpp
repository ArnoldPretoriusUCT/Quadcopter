/*
 * RATECONTROL.cpp
 *
 *  Created on: 13 Feb 2018
 *      Author: Arnold
 */
#include "RATECONTROL.h"

float RATECONTROL::r[3],
      RATECONTROL::u[3],
      RATECONTROL::e[3],
      RATECONTROL::rollState[4],
      RATECONTROL::pitchState[4],
	  RATECONTROL::yawState[4],
	  RATECONTROL::loopFrequency;
bool  RATECONTROL::isClosedLoop;

void RATECONTROL::initialise()
{
	r[0] = r[1] = r[2] = 0;
}

void RATECONTROL::iterate()
{
	if( !RATECONTROL::isClosedLoop )
	{
		memset(&rollState[0],0,8);
		memset(&pitchState[0],0,8);
		memset(&yawState[0],0,8);
	}

	e[0] = r[0] - IMU::gyro[0];
	e[1] = r[1] - IMU::gyro[1];
	e[2] = r[2] - IMU::gyro[2];

	u[0] = RATECONTROL_PHI_K*e[0];
	u[1] = RATECONTROL_THETA_K*e[1];
	u[2] = PIc(e[2],RATECONTROL_PSI_K,RATECONTROL_PSI_WZ,dT,RATECONTROL_PSI_UMAX, &yawState[0]);

//	Serial.print(u[0]);
//	Serial.print(", ");
//	Serial.print(u[1]);
//	Serial.print(", ");
//	Serial.println(u[2]);
}



