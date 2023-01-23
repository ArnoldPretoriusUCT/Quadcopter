/*
 * QUATERNIONEKF.cpp
 *
 *  Created on: 25 Nov 2016
 *      Author: Arnold
 */
#include "QUATERNIONEKF.h"

float QUATERNIONEKF::cameraRotationRate[3],
	  QUATERNIONEKF::cameraQuaternion[4],
	  QUATERNIONEKF::cameraCovariance[4];



