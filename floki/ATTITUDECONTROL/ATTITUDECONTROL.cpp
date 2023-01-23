/*
 * ATTITUDECONTROL.cpp
 *
 *  Created on: 10 Aug 2016
 *      Author: Arnold
 */
#include "ATTITUDECONTROL.H"

bool ATTITUDECONTROL::isClosedLoop,
	 ATTITUDECONTROL::controlMode;

float ATTITUDECONTROL::u[3],
	  ATTITUDECONTROL::e[3],
	  ATTITUDECONTROL::uz[2],
	  ATTITUDECONTROL::loopFrequency,
	  ATTITUDECONTROL::rateReference[3],
	  ATTITUDECONTROL::angleReference[3],
	  ATTITUDECONTROL::Kq[3],
	  ATTITUDECONTROL::Kw[3],
	  ATTITUDECONTROL::qReference[4],
	  ATTITUDECONTROL::controllerState[3];

void ATTITUDECONTROL::initialise()
{
	Kq[0] = ATTITUDECONTROL_KQ_X;
	Kq[1] = ATTITUDECONTROL_KQ_Y;
	Kq[2] = ATTITUDECONTROL_KQ_Z;
	Kw[0] = ATTITUDECONTROL_KW_X;
	Kw[1] = ATTITUDECONTROL_KW_Y;
	Kw[2] = ATTITUDECONTROL_KW_Z;

	qReference[0] = 1;
	isClosedLoop = false;
	loopFrequency = 0;
}

void ATTITUDECONTROL::iterate()
{
	//	    		----	   e   ----	  u  ----   w    ---  dq  -----  q			Gq - Proportional gain
	// q_r -> O -> | Kq | -> O -> | Kw | -> | Pw | ---> | M | -> | 1/s | --->	    Gw - Proportional gain
	//		 *^	    ----     ^     ----      ----   |    ---      -----   |			Pw - Angular rate plant
	//        |              |			 -----	    |			 	      |			M  - Angular rate to quaternion rate mapping
	//		  |			   	 \----------| LPF | <---/			          |			LPF - 2nd order @100Hz
	//        |                          -----                            |
	//  	  \-----------------------------------------------------------/

	if( isClosedLoop )
	{
		getError();
		u[0] = Kw[0]*( Kq[0]*e[0]-ECF::gyro[0] );
		u[1] = Kw[1]*( Kq[1]*e[1]-ECF::gyro[1] );
		u[2] = PIc(-ECF::gyro[2], .005, 0.1, dT, 1, &controllerState[0]);
	}
}

void ATTITUDECONTROL::getError()
{
	qReference[0] = cosf( asinf( sqrt(qReference[1]*qReference[1]+qReference[2]*qReference[2]+qReference[3]*qReference[3]) ) );

	float axb[3],qconj[4],qerr[4];
	QuaternionMath::conjugate(ECF::qFused,&qconj[0]);
	QuaternionMath::multiply(qconj,qReference,&qerr[0]);

	float qe_o = qerr[0];
	float qe_x = qerr[1];
	float qe_y = qerr[2];
	float qe_z = qerr[3];
	float v[3];
	v[0] = 2*(qe_x*qe_z+qe_o*qe_y);
	v[1] = 2*(qe_y*qe_z-qe_o*qe_x);
	v[2] = 1-2*(qe_x*qe_x+qe_y*qe_y);

	float vo[] = {0,0,1};
	cross(vo,v,&axb[0]);

	float alpha = asinf( sqrt( axb[0]*axb[0]+axb[1]*axb[1]+axb[2]*axb[2] ) );
	if( alpha == 0 )
	{
		e[0] = e[1] = e[2] = 0;
	}
	else
	{
		e[0] = axb[0]*(alpha/sinf(alpha));
		e[1] = axb[1]*(alpha/sinf(alpha));
		e[2] = axb[2]*(alpha/sinf(alpha)); //always zero
	}
}

void ATTITUDECONTROL::getLoopFrequency()
{
	static uint32_t tLoop_past = 0;
	uint32_t * pntr = &tLoop_past;

	float dt = getDT(pntr);
	if( dt != 0 )
	{
		loopFrequency = 1e6/dt;
	}
	else loopFrequency = 0;
//	Serial.println(dt);
}



