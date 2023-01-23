/*
 * QuaternionMath.cpp
 *
 *  Created on: 03 Dec 2015
 *      Author: Arnold
 */
#include "QuaternionMath.h"

QuaternionMath::QuaternionMath()
{

}

float QuaternionMath::getNorm(const float quaternion[4])
{
	float quaternionNorm = sqrt( pow(quaternion[0],2) + pow(quaternion[1],2) + pow(quaternion[2],2) + pow(quaternion[3],2) );
	return quaternionNorm;
}

void QuaternionMath::normalise(float quaternion[4])
{
	float quaternionNorm = sqrt( pow(quaternion[0],2) + pow(quaternion[1],2) + pow(quaternion[2],2) + pow(quaternion[3],2) );
	for(int i=0;i<4;i++)
	{
		quaternion[i] = quaternion[i]/quaternionNorm;
	}
}

void QuaternionMath::differentiate(const float q[4], float q_past[4], const float dt, float dq[4])
{
	dq[0] = (q[0] - q_past[0])/dt;
	q_past[0] = q[0];

	dq[1] = (q[1] - q_past[1])/dt;
	q_past[1] = q[1];

	dq[2] = (q[2] - q_past[2])/dt;
	q_past[2] = q[2];

	dq[3] = (q[3] - q_past[3])/dt;
	q_past[3] = q[3];
}

void QuaternionMath::getQuaternionFromVectors(const float vb[3], const float vi[3], float quat[4])
{
	cross(vb, vi, &quat[1]);
	quat[0] = 1 + vi[0]*vb[0] + vi[1]*vb[1] + vi[2]*vb[2];

	normalise(quat);
}

void QuaternionMath::getRotationMatrixFromQuaternion(const float q[4], float R[9])
{
	R[0] =  (1 - 2*q[2]*q[2]);		R[1] = 2*q[1]*q[2]; 		R[2] =  2*q[0]*q[2];
	R[3] =     2*q[1]*q[2]; 		R[4] = (1-2*q[1]*q[1]);		R[5] = -2*q[0]*q[1];
	R[6] =    -2*q[0]*q[2]; 		R[7] = 2*q[0]*q[1];    		R[8] =  (1 - 2*(q[1]*q[1] + q[2]*q[2])); //(3x3)
}

void QuaternionMath::multiply(const float q[4], const float p[4], float qp[4])
{
	float qTemp[4];
	qTemp[0] = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3];
	qTemp[1] = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2];
	qTemp[2] = q[0]*p[2] - q[1]*p[3] + q[2]*p[0] + q[3]*p[1];
	qTemp[3] = q[0]*p[3] + q[1]*p[2] - q[2]*p[1] + q[3]*p[0];
	memcpy(&qp[0],&qTemp[0],sizeof(qTemp));
}

void QuaternionMath::conjugate(const float q[4], float qConj[4])
{
	qConj[0] =  q[0];
	qConj[1] = -q[1];
	qConj[2] = -q[2];
	qConj[3] = -q[3];
}

void QuaternionMath::getXYQuaternion(const float q_xyz[4], float q_xy[4])
{
	q_xy[0] = 1/(sqrt(1-q_xyz[1]*q_xyz[1]-q_xyz[2]*q_xyz[2]))*(q_xyz[0]*q_xyz[0]+q_xyz[3]*q_xyz[3]);
	q_xy[1] = 1/(sqrt(1-q_xyz[1]*q_xyz[1]-q_xyz[2]*q_xyz[2]))*(q_xyz[0]*q_xyz[1]+q_xyz[2]*q_xyz[3]);
	q_xy[2] = 1/(sqrt(1-q_xyz[1]*q_xyz[1]-q_xyz[2]*q_xyz[2]))*(q_xyz[0]*q_xyz[2]-q_xyz[1]*q_xyz[3]);
	q_xy[3] = 0;

//	q_xy[0] = q_xyz[0];
//	q_xy[1] = q_xyz[1];
//	q_xy[2] = q_xyz[2];
//	q_xy[3] = 0;

	normalise(q_xy);
}

void QuaternionMath::rotateVector(const float v[3], const float q[4], float v_rotated[3])
{
	float v_temp[4],preQuat[4],postQuat[4],q_conj[4];
	v_temp[0] = 0;
	v_temp[1] = v[0];
	v_temp[2] = v[1];
	v_temp[3] = v[2];

	multiply(q,v_temp,&preQuat[0]);
	conjugate(q,&q_conj[0]);
	multiply(preQuat,q_conj,&v_temp[0]);
//	QuaternionMath::normalise(v_temp);
	memcpy(&v_rotated[0],&v_temp[0],sizeof(v_rotated));
}
