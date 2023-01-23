/*
 * MOTORSPEEDCONTROL.cpp
 *
 *  Created on: 11 Aug 2016
 *      Author: Arnold
 */
#include "MOTORSPEEDCONTROL.h"

bool MOTORSPEEDCONTROL::isClosedLoop,
     MOTORSPEEDCONTROL::isSaturated,
     MOTORSPEEDCONTROL::torqueFeedForward,
     MOTORSPEEDCONTROL::dwEnabled;
float MOTORSPEEDCONTROL::reference,
	  MOTORSPEEDCONTROL::batteryVoltage,
	  MOTORSPEEDCONTROL::K[4],
	  MOTORSPEEDCONTROL::w,
	  MOTORSPEEDCONTROL::dw,
	  MOTORSPEEDCONTROL::r,
	  MOTORSPEEDCONTROL::b[4],
	  MOTORSPEEDCONTROL::u_sat[4],
	  MOTORSPEEDCONTROL::u_blade[4];


void MOTORSPEEDCONTROL::initialise()
{
	isClosedLoop = isSaturated = false;
	reference = 0;
	r = 0;
//	K = MOTORSPEEDCONTROL_K_INIT;
	w = MOTORSPEEDCONTROL_W_ZERO;
}

void MOTORSPEEDCONTROL::iterate()
{
	ENCODER::getMotorSpeed();
	getBatteryVoltage();

	if( RADIO::isOpen && WIXEL::isOpen && !WATCHDOG::isCritical )
	{
		if( !isClosedLoop )
		{
			if( PWM::isCalibrating )
			{
				PWM::escCom[0] = MOTORSPEEDCONTROL_PWM_MIN*(1+reference/800);
				PWM::escCom[1] = MOTORSPEEDCONTROL_PWM_MIN*(1+reference/800);
				PWM::escCom[2] = MOTORSPEEDCONTROL_PWM_MIN*(1+reference/800);
				PWM::escCom[3] = MOTORSPEEDCONTROL_PWM_MIN*(1+reference/800);
			}
			else
			{
				PWM::escCom[0] = PWM::escCom[1] = PWM::escCom[2] = PWM::escCom[3] = MOTORSPEEDCONTROL_PWM_MIN*(1+(HEAVECONTROL::u-HEAVECONTROL_THRUST_MIN)/(HEAVECONTROL_THRUST_MAX-HEAVECONTROL_THRUST_MIN));
			}
			u_sat[0] = u_sat[1] = u_sat[2] = u_sat[3] = b[0] = b[1] = b[2] = b[3] = r = 0;

		}
		else //isClosedLoop
		{
			if( PWM::motorsArmed )
//			if( HEAVECONTROL::u > 1.2*HEAVECONTROL_THRUST_MIN )
			{
				r = r * expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_UP) + ( 1 - expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_UP) ) * reference;
				b[0] = b[0] * expf(-w*dT) + ( 1 - expf(-w*dT) )  * u_sat[0];
				b[1] = b[1] * expf(-w*dT) + ( 1 - expf(-w*dT) )  * u_sat[1];
				b[2] = b[2] * expf(-w*dT) + ( 1 - expf(-w*dT) )  * u_sat[2];
				b[3] = b[3] * expf(-w*dT) + ( 1 - expf(-w*dT) )  * u_sat[3];

				u_sat[0] = r - ENCODER::motorSpeed[0] + b[0];
				u_sat[1] = r - ENCODER::motorSpeed[1] + b[1];
				u_sat[2] = r - ENCODER::motorSpeed[2] + b[2];
				u_sat[3] = r - ENCODER::motorSpeed[3] + b[3];


				for(int i=0;i<4;i++)
				{
					if( fabsf(reference-ENCODER::motorSpeed[i]) > MOTORSPEEDCONTROL_W_DELTA )
					{
						K[i] = MOTORSPEEDCONTROL_K_INIT;
					}
					else
					{
						K[i] = K[i]+MOTORSPEEDCONTROL_K_DELTA;
						if( K[i] >= MOTORSPEEDCONTROL_K_NOM)
						{
							K[i] = MOTORSPEEDCONTROL_K_NOM;
						}
					}
				}

				for(int i=0;i<4;i++)
				{
					if( u_sat[i] >= (MOTORSPEEDCONTROL_PWM_MAX-MOTORSPEEDCONTROL_PWM_MIN)/(K[i]/w) )
					{
						u_sat[i] = (MOTORSPEEDCONTROL_PWM_MAX-MOTORSPEEDCONTROL_PWM_MIN)/(K[i]/w);
					}
					if( u_sat[i] <= 0 )
					{
						u_sat[i] = 0;
					}

				}

//				if( MOTORSPEEDCONTROL::torqueFeedForward == true )
//				{
//					u_blade[0] = MOTORSPEEDCONTROL_FEEDFORWARD_GAIN*((PWM::servoComDelta[0]-PWM_SERVO_DELTA_INIT)+(PWM::servoComDelta[2]-PWM_SERVO_DELTA_INIT));
//					u_blade[1] = MOTORSPEEDCONTROL_FEEDFORWARD_GAIN*((PWM::servoComDelta[1]-PWM_SERVO_DELTA_INIT)+(PWM::servoComDelta[3]-PWM_SERVO_DELTA_INIT));
//				}
				if(torqueFeedForward)
				{
					for(int i=0;i<4;i++)
					{
						u_blade[i] = MOTORSPEEDCONTROL_FEEDFORWARD_GAIN*PWM::servoComDelta[i];
					}
				}
				else
				{
					u_blade[0] = u_blade[1] = u_blade[2] = u_blade[3] = 0;
				}

				static float u_lpf[] = {0,0,0,0};
				static float lpf_stateVec[] = {0,0,0,0,0,0,0,0};

				for( int i=0;i<4;i++ )
				{
					u_lpf[i] =  LPF(u_sat[i], dT, MOTORSPEEDCONTROL_W_POLE, &lpf_stateVec[2*i]);
					PWM::escCom[i] = MOTORSPEEDCONTROL_PWM_MIN + (K[i]/w) * u_lpf[i] + u_blade[i];
				}

//				PWM::escCom[0] = MOTORSPEEDCONTROL_PWM_MIN + (K[0]/w) * u_sat[0] + u_blade[0];
//				PWM::escCom[1] = MOTORSPEEDCONTROL_PWM_MIN + (K[1]/w) * u_sat[1] + u_blade[1];
//				PWM::escCom[2] = MOTORSPEEDCONTROL_PWM_MIN + (K[2]/w) * u_sat[2] + u_blade[2];
//				PWM::escCom[3] = MOTORSPEEDCONTROL_PWM_MIN + (K[3]/w) * u_sat[3] + u_blade[3];
			}
			else
			{
				r = u_sat[0] = u_sat[1] = u_sat[2] = u_sat[3] = b[0] = b[1] = b[2] = b[3] = 0;
				PWM::escCom[0] = PWM::escCom[0] * expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) + ( 1 - expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) ) * MOTORSPEEDCONTROL_PWM_MIN;
				PWM::escCom[1] = PWM::escCom[1] * expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) + ( 1 - expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) ) * MOTORSPEEDCONTROL_PWM_MIN;
				PWM::escCom[2] = PWM::escCom[2] * expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) + ( 1 - expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) ) * MOTORSPEEDCONTROL_PWM_MIN;
				PWM::escCom[3] = PWM::escCom[3] * expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) + ( 1 - expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) ) * MOTORSPEEDCONTROL_PWM_MIN;
			}
		}
	}
	else
	{
		PWM::escCom[0] = PWM::escCom[0] * expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) + ( 1 - expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) ) * MOTORSPEEDCONTROL_PWM_MIN;
		PWM::escCom[1] = PWM::escCom[1] * expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) + ( 1 - expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) ) * MOTORSPEEDCONTROL_PWM_MIN;
		PWM::escCom[2] = PWM::escCom[2] * expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) + ( 1 - expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) ) * MOTORSPEEDCONTROL_PWM_MIN;
		PWM::escCom[3] = PWM::escCom[3] * expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) + ( 1 - expf(-dT*MOTORSPEEDCONTROL_PREFILTER_W_SPOOL_DOWN) ) * MOTORSPEEDCONTROL_PWM_MIN;
	}

	for( int i=0;i<4;i++)
	{
		if(PWM::escCom[i] > MOTORSPEEDCONTROL_PWM_MAX)
		{
			PWM::escCom[i] = MOTORSPEEDCONTROL_PWM_MAX;
		}
		if(PWM::escCom[i] < MOTORSPEEDCONTROL_PWM_MIN)
		{
			PWM::escCom[i] = MOTORSPEEDCONTROL_PWM_MIN;
		}
		if(PWM::escCom[0] > MOTORSPEEDCONTROL_PWM_SATURATION)
		{
			isSaturated = true;
		}
		else
		{
			isSaturated = false;
		}
	}
}

void MOTORSPEEDCONTROL::getBatteryVoltage()
{
	batteryVoltage = (float)analogRead(A0)/65535*(12.4876/11.4*12);
//	Serial.println(batteryVoltage);
}

