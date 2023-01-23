/*
 * THRUSTCONTROL.cpp
 *
 *  Created on: 10 Feb 2020
 *      Author: ArnoldPretorius
 */

#include "THRUSTCONTROL.h"

float THRUSTCONTROL::current[4],
	  THRUSTCONTROL::thrustEstimate[4],
	  THRUSTCONTROL::thrustReference[4],
	  THRUSTCONTROL::servoCommand[4];

void THRUSTCONTROL::initialise()
{
 //
}

void THRUSTCONTROL::iterate()
{
	getThrustEstimates();
	static float state[12];
	for(int i=0;i<4;i++)
	{
		thrustReference[i] = (250.0/17)*(HEAVECONTROL::u-3)+50;
		float e = thrustReference[i]-thrustEstimate[i];
		if( fabsf(MOTORSPEEDCONTROL::reference-ENCODER::motorSpeed[i]) < THRUSTCONTROL_SPEED_THRESHOLD )
		{
			servoCommand[i] = PID(e,THRUSTCONTROL_K,THRUSTCONTROL_W_ZERO1,THRUSTCONTROL_W_ZERO2,THRUSTCONTROL_W_POLE,dT, &state[3*i]);
			servoCommand[i] = servoCommand[i]+THRUSTCONTROL_X*thrustReference[i];
		}
		else
		{
			memset(&state[3*i],0,3*4);
			servoCommand[i] = THRUSTCONTROL_X*thrustReference[i]; //open-loop
		}
	}
}

void THRUSTCONTROL::getThrustEstimates()
{
	float temp[4];
	temp[0] = THRUSTCONTROL_CURRENT_GAIN*( (float)analogRead(A13)/65535*3.3-.5 );
	temp[1] = THRUSTCONTROL_CURRENT_GAIN*( (float)analogRead(A10)/65535*3.3-.5 );
	temp[2] = THRUSTCONTROL_CURRENT_GAIN*( (float)analogRead(A11)/65535*3.3-.5 );
	temp[3] = THRUSTCONTROL_CURRENT_GAIN*( (float)analogRead(A12)/65535*3.3-.5 );

	static float buffer[16];
	current[0] = -0.1+LPF2(temp[0], dT, THRUSTCONTROL_ESTIMATOR_CUTOFF_FREQUENCY, &buffer[0], &buffer[2]);
	current[1] = -0.1+LPF2(temp[1], dT, THRUSTCONTROL_ESTIMATOR_CUTOFF_FREQUENCY, &buffer[4], &buffer[6]);
	current[2] =  0.0+LPF2(temp[2], dT, THRUSTCONTROL_ESTIMATOR_CUTOFF_FREQUENCY, &buffer[8], &buffer[10]);
	current[3] = -0.1+LPF2(temp[3], dT, THRUSTCONTROL_ESTIMATOR_CUTOFF_FREQUENCY, &buffer[12], &buffer[14]);

	for(int i=0;i<4;i++)
	{
		thrustEstimate[i] =  THRUSTCONTROL_B4*powf(current[i],4)+THRUSTCONTROL_B3*powf(current[i],3)+THRUSTCONTROL_B2*powf(current[i],2)+THRUSTCONTROL_B1*current[i]+THRUSTCONTROL_B0;
	}
//	Serial.println(thrustReference[2]);

}

