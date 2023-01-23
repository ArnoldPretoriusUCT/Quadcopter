/*
 * ECF.cpp
 *
 *  Created on: 5 Sep 2018
 *      Author: Arnold
 */
#include "ECF.h"

float ECF::qFused[4],
	  ECF::qCamera[4],
	  ECF::gyro[3];

void ECF::initialise()
{
	qFused[0] = 1; qFused[1] = 0; qFused[2] = 0; qFused[3] = 0;
	qCamera[0] = 1; qCamera[1] = 0; qCamera[2] = 0; qCamera[3] = 0;
}

void ECF::iterate()
{
	//get scalar of qCamera
	qCamera[0] = cosf( asinf(sqrtf(qCamera[1]*qCamera[1]+qCamera[2]*qCamera[2]+qCamera[3]*qCamera[3])) );
	//get BF "quaternion error"
	float qConj[4],qError[4];
	QuaternionMath::conjugate(&qFused[0],&qConj[0]);
	QuaternionMath::multiply(&qConj[0],&qCamera[0],&qError[0]);
	//get augmented rate signal
	float K;
	if(WIXEL::newData){ K=2; }	else{ K=0; } //add valid data condition?
	WIXEL::newData = false;

	gyro[0] = K*qError[1]+IMU::gyro[0];
	gyro[1] = K*qError[2]+IMU::gyro[1];
	gyro[2] = K*qError[3]+IMU::gyro[2];
	//propagate quaternion with fused rate signal
	qFused[0] = qFused[0]-dT/2*gyro[0]*qFused[1]-dT/2*gyro[1]*qFused[2]-dT/2*gyro[2]*qFused[3];
	qFused[1] = qFused[1]+dT/2*gyro[0]*qFused[0]+dT/2*gyro[2]*qFused[2]-dT/2*gyro[1]*qFused[3];
	qFused[2] = qFused[2]+dT/2*gyro[1]*qFused[0]-dT/2*gyro[2]*qFused[1]+dT/2*gyro[0]*qFused[3];
	qFused[3] = qFused[3]+dT/2*gyro[2]*qFused[0]+dT/2*gyro[1]*qFused[1]-dT/2*gyro[0]*qFused[2];
	//normalise quaternion
	float qNorm = sqrtf(qFused[0]*qFused[0]+qFused[1]*qFused[1]+qFused[2]*qFused[2]+qFused[3]*qFused[3]);
	qFused[0] = qFused[0]/qNorm;
	qFused[1] = qFused[1]/qNorm;
	qFused[2] = qFused[2]/qNorm;
	qFused[3] = qFused[3]/qNorm;

}



