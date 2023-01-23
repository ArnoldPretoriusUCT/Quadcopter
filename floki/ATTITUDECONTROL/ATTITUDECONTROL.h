/*
 * ATTITUDECONTROL.h
 *
 *  Created on: 10 Aug 2016
 *      Author: Arnold
 */

#ifndef ATTITUDECONTROL_H_
#define ATTITUDECONTROL_H_

#include "math.h"
#include "QuaternionMath.h"
#include "RADIO.h"
#include "IMU.h"
#include "ECF.h"

#define PHI_REF_LIMIT 60.0
#define THETA_REF_LIMIT 60.0
#define PSI_REF_LIMIT 10.0

#define ATTITUDECONTROL_KQ_X 5
#define ATTITUDECONTROL_KQ_Y 5
#define ATTITUDECONTROL_KQ_Z 0
#define ATTITUDECONTROL_KW_X .08
#define ATTITUDECONTROL_KW_Y .08
#define ATTITUDECONTROL_KW_Z .16
class ATTITUDECONTROL
{
	public:
		static void initialise();
		static void iterate();

		static float u[3],rateReference[3],angleReference[3],loopFrequency,Kq[3],Kw[3];
		static float uz[2];
		static bool isClosedLoop,controlMode;

		static float qReference[4];

	private:
		static void getError();
		static void getLoopFrequency();
		static float e[3];
		static float controllerState[3];
};



#endif /* ATTITUDECONTROL_H_ */
