/*
 * PWM.cpp
 *
 *  Created on: 03 Dec 2015
 *      Author: Arnold
 */
#include "PWM.h"

float PWM::ledBlobCom,
	  PWM::escCom[4],
	  PWM::servoCom[4],
	  PWM::servoOffset[4],
	  PWM::servoComDelta[4],
	  PWM::servoComLowerLimit[4],
	  PWM::servoComUpperLimit[4],
	  PWM::servoCalibrationCom,
	  PWM::U[4];

bool PWM::isCalibrating,
	 PWM::servoCentering,
	 PWM::motorsArmed,
	 PWM::radioClear,
	 PWM::criticalFailure;

void PWM::initialise()
{
	escCom[0] = escCom[1] = escCom[2] = escCom[3] = 4500;

	servoOffset[0] = PWM_SERVO1_OFFSET;
	servoOffset[1] = PWM_SERVO2_OFFSET;
	servoOffset[2] = PWM_SERVO3_OFFSET;
	servoOffset[3] = PWM_SERVO4_OFFSET;

	for( int i=0;i<4;i++ )
	{
		servoComLowerLimit[i] = servoOffset[i] - PWM_SERVO_DELTA_MAX;
		servoComUpperLimit[i] = servoOffset[i] + PWM_SERVO_DELTA_MAX;
	}

	servoCom[0] = servoOffset[0];
	servoCom[1] = servoOffset[1];
	servoCom[2] = servoOffset[2];
	servoCom[3] = servoOffset[3];

	criticalFailure = false;
	radioClear = false;
	isCalibrating = false;
	motorsArmed = false;
	servoCalibrationCom = 1.8495e4;
	ledBlobCom = 0;

	pinMode(PWM_SERVO1, OUTPUT);
	pinMode(PWM_SERVO2, OUTPUT);
	pinMode(PWM_SERVO3, OUTPUT);
	pinMode(PWM_SERVO4, OUTPUT);

	pinMode(PWM_ESC1, OUTPUT);
	pinMode(PWM_ESC2, OUTPUT);

	analogWriteFrequency(PWM_ESC1, PWM_FREQUENCY);
	analogWriteFrequency(PWM_ESC2, PWM_FREQUENCY);
	analogWriteFrequency(PWM_ESC3, PWM_FREQUENCY);
	analogWriteFrequency(PWM_ESC4, PWM_FREQUENCY);
	analogWriteFrequency(PWM_SERVO1, PWM_FREQUENCY);
	analogWriteFrequency(PWM_SERVO2, PWM_FREQUENCY);
	analogWriteFrequency(PWM_SERVO3, PWM_FREQUENCY);
	analogWriteFrequency(PWM_SERVO4, PWM_FREQUENCY);

	analogWriteRes(PWM_ANALOG_RES);

	analogWrite(PWM_SERVO1, servoCom[0]);
	analogWrite(PWM_SERVO2, servoCom[1]);
	analogWrite(PWM_SERVO3, servoCom[2]);
	analogWrite(PWM_SERVO4, servoCom[3]);
}

void PWM::write()
{
//	analogWrite(PWM_SERVO1, (uint16_t)servoCom[0]);
//	analogWrite(PWM_SERVO2, (uint16_t)servoCom[1]);
	analogWrite(PWM_SERVO3, (uint16_t)servoCom[2]);
//	analogWrite(PWM_SERVO4, (uint16_t)servoCom[3]);

	if( motorsArmed && !WATCHDOG::isCritical )
	{
//		analogWrite(PWM_ESC1, (uint16_t)escCom[0]);
//		analogWrite(PWM_ESC2, (uint16_t)escCom[1]);
		analogWrite(PWM_ESC3, (uint16_t)escCom[2]);
//		analogWrite(PWM_ESC4, (uint16_t)escCom[3]);
	}
	else
	{
		analogWrite(PWM_ESC1, MOTORSPEEDCONTROL_PWM_MIN);
		analogWrite(PWM_ESC2, MOTORSPEEDCONTROL_PWM_MIN);
		analogWrite(PWM_ESC3, MOTORSPEEDCONTROL_PWM_MIN);
		analogWrite(PWM_ESC4, MOTORSPEEDCONTROL_PWM_MIN);
	}
}

void PWM::servoMapping()
{
	static const float l = 0.19; 						// distance from COM to rotor axis [m]
	static const float a = (3.5e-3*.677); 				// servoCom to rotorForce mapping  [N/16bitCnt]
	static const float b = (2.4375e-08*.677*.677);		// servoCom to rotorTorque mapping [Nm/16bitCnt]

	U[0] = HEAVECONTROL::u;
	if( ATTITUDECONTROL::isClosedLoop )
	{
		U[1] = ATTITUDECONTROL::u[0];
		U[2] = ATTITUDECONTROL::u[1];
		U[3] = ATTITUDECONTROL::u[2];
	}
	else
	{
		U[1] = U[2] = U[3] = 0;
	}

	if( RADIO::isOpen && WIXEL::isOpen  )
	{
		if( !isCalibrating )
		{//
			if( U[0] != 0)
			{
				//nonlinear mapping: Input->Blade angle (at nominal hover)
				servoComDelta[0] = ( 2*U[3]*powf(a,2)*powf(l,2) + b*powf(l,2)*powf(U[0],2) - 2*b*l*U[0]*U[2] + b*powf(U[1],2) - b*powf(U[2],2))/(4*a*b*powf(l,2)*U[0]);
				servoComDelta[1] = (-2*U[3]*powf(a,2)*powf(l,2) + b*powf(l,2)*powf(U[0],2) - 2*b*l*U[0]*U[1] - b*powf(U[1],2) + b*powf(U[2],2))/(4*a*b*powf(l,2)*U[0]);
				servoComDelta[2] = ( 2*U[3]*powf(a,2)*powf(l,2) + b*powf(l,2)*powf(U[0],2) + 2*b*l*U[0]*U[2] + b*powf(U[1],2) - b*powf(U[2],2))/(4*a*b*powf(l,2)*U[0]);
				servoComDelta[3] = (-2*U[3]*powf(a,2)*powf(l,2) + b*powf(l,2)*powf(U[0],2) + 2*b*l*U[0]*U[1] - b*powf(U[1],2) + b*powf(U[2],2))/(4*a*b*powf(l,2)*U[0]);

				//sign change for servos 1 and 3 (because of physical servo mounting)
				servoComDelta[0] = -servoComDelta[0];
				servoComDelta[2] = -servoComDelta[2];

				for( int i=0;i<4;i++ )
				{
					if( fabs(servoComDelta[i]) > PWM_SERVO_DELTA_MAX )
					{
						servoComDelta[i] = sign(servoComDelta[i])*PWM_SERVO_DELTA_MAX;
					}
				}
			}
			else
			{
				servoComDelta[0] = servoComDelta[1] = servoComDelta[2] = servoComDelta[3] = 0;
			}
		}
		else //isCalibrating
		{
			for( int i=0;i<4;i++ )
			{
				servoComDelta[i] = (HEAVECONTROL::u-11.8)*120*3.4;
//				servoComDelta[i] = powf(-1,i+1)*THRUSTCONTROL::servoCommand[i];
				if( ATTITUDECONTROL::qReference[2] > 0.1 )
				{
					servoComDelta[i] += 500;
				}
				else if( ATTITUDECONTROL::qReference[2] < -0.1 )
				{
					servoComDelta[i] -= 500;
				}
			}
			Serial.println(servoComDelta[2]);
		}
		for(int i=0;i<4;i++ )
		{
			if( fabs(servoComDelta[i]) >= PWM_SERVO_DELTA_MAX )
			{
				servoComDelta[i] = sign(servoComDelta[i])*PWM_SERVO_DELTA_MAX;
			}
			servoCom[i] = servoComDelta[i] + servoOffset[i];
		}
	}
	else
	{
		for(int i=0;i<4;i++)
		{
			servoCom[i] = servoOffset[i];
		}
	}
}


