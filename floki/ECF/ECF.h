/*
 * ECF.h
 *
 *  Created on: 5 Sep 2018
 *      Author: Arnold
 */

#ifndef ECF_H_
#define ECF_H_

#include "Arduino.h"
#include "QuaternionMath.h"
#include "IMU.h"
#include "WIXEL.h"

#define ECF_K 4

class ECF
{
	public:
		static void initialise();
		static void iterate();

		static float qFused[4],qCamera[4],gyro[3];
	private:
};


#endif /* ECF_H_ */
