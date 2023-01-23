/*
 * QUATERNIONEKF.h
 *
 *  Created on: 25 Nov 2016
 *      Author: Arnold
 */

#ifndef QUATERNIONEKF_H_
#define QUATERNIONEKF_H_

class QUATERNIONEKF
{
	public:
		static float cameraRotationRate[3],cameraQuaternion[4],cameraCovariance[4];
};


#endif /* QUATERNIONEKF_H_ */
