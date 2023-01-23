/*
 * RATECONTROL.h
 *
 *  Created on: 13 Feb 2018
 *      Author: Arnold
 */

#ifndef RATECONTROL_H_
#define RATECONTROL_H_

#include "math.h"
#include "common.h"
#include "RADIO.h"
#include "IMU.h"

#define RATECONTROL_PHI_K 		0.08
#define RATECONTROL_THETA_K 	0.08
#define RATECONTROL_PSI_K		0.015
#define RATECONTROL_PSI_WZ		0.20
#define RATECONTROL_PSI_UMAX    0.25

class RATECONTROL
{
	public:
		static void initialise();
		static void iterate();

		static float u[3],r[3],e[3],loopFrequency;
		static bool isClosedLoop;

	private:
		static float rollState[4],pitchState[4],yawState[4];
};

#endif /* RATECONTROL_H_ */
