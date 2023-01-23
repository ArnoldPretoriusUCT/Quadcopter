/*
 * QuaternionMath.h
 *
 *  Created on: 03 Dec 2015
 *      Author: Arnold
 */

#ifndef QUATERNIONMATH_H_
#define QUATERNIONMATH_H_

#include "math.h"
#include "common.h"

class QuaternionMath
{
	public:
		QuaternionMath();
		static float getNorm(const float quaternion[4]);
		static void normalise(float quaternion[4]);
		static void differentiate(const float q[4], float q_past[4], const float dt, float dq[4]);
		static void getQuaternionFromVectors(const float vb[3], const float vi[3], float quat[4]);
		static void getRotationMatrixFromQuaternion(const float q[4], float R[9]);
		static void multiply(const float q[4], const float p[4], float qp[4]);
		static void conjugate(const float q[4], float qConj[4]);
		static void getXYQuaternion(const float q_xyz[4], float q_xy[4]);
		static void rotateVector(const float v[3], const float q[4], float v_rotated[3]);
};


#endif /* QUATERNIONMATH_H_ */
