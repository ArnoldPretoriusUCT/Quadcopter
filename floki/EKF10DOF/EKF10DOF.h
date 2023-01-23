/*
 * EKF10DOF.h
 *
 *  Created on: 04 Apr 2016
 *      Author: Arnold
 */

#ifndef EKF10DOF_H_
#define EKF10DOF_H_

#include "MatrixMath.h"
#include "common.h"
#include "stdlib.h"
#include "Arduino.h"
#include "ADNS3080.h"

//#define EKF10DOF_COM_2_USM 6e-2

//Process noise covariance diagonal
#define EKF10DOF_Q11 		3.7946e-07//powf(6.16e-04,2) 	//(rad/s)^2 - gyro x
#define EKF10DOF_Q22 		3.7946e-07//powf(6.16e-04,2) 	//(rad/s)^2 - gyro y
#define EKF10DOF_Q33 		3.7946e-07//powf(6.16e-04,2) 	//(rad/s)^2 - gyro z
#define EKF10DOF_Q44 		7.8400e-04//powf(28.0e-03,2) 	//(m/s/s)^2 - accel x
#define EKF10DOF_Q55 		7.8400e-04//powf(28.0e-03,2) 	//(m/s/s)^2 - acce; y
#define EKF10DOF_Q66 		7.8400e-04//powf(28.0e-03,2)	//(m/s/s)^2 - accel z
#define EKF10DOF_Q77 		1e-2//powf(1.0e-01,2)	//(m/s/s)^2 - accel bias x
#define EKF10DOF_Q88 		1e-2//powf(1.0e-01,2) 	//(m/s/s)^2 - accel bias y
#define EKF10DOF_Q99 		1e-2//powf(1.0e-01,2) 	//(m/s/s)^2 - accel bias z
//Measurement noise covariance diagonal
#define EKF10DOF_R11 		4.0e-6//powf(2.0e-03,2) 	//(m/s)^2 - accel x
#define EKF10DOF_R22 		4.0e-6//powf(2.0e-03,2) 	//(m/s)^2 - accel y
#define EKF10DOF_R33 		4.0e-6//powf(3.0e-03,2) 	//m^2     - accel z
//Initial error covariance matrix diagonal
#define EKF10DOF_P11   		1.0e-08			//altitude
#define EKF10DOF_P22   		2.0e-07			//velocity x
#define EKF10DOF_P33   		2.0e-07			//velocity y
#define EKF10DOF_P44   		2.0e-07			//velocity z
#define EKF10DOF_P55   		1.0e-07			//quat o
#define EKF10DOF_P66   		1.0e-07			//quat x
#define EKF10DOF_P77   		1.0e-07			//quat y
#define EKF10DOF_P88   		1.0e-6//powf(1.0e-3,2)	//accel bias x
#define EKF10DOF_P99   		1.0e-6//powf(1.0e-3,2)	//accel bias y
#define EKF10DOF_P1010 		1.0e-6//powf(1.0e-3,2)	//accel bias z

#define EKF10DOF_PZ_INIT	0.03
#define EKF10DOF_VX_INIT 	0.0
#define EKF10DOF_VY_INIT 	0.0
#define EKF10DOF_VZ_INIT 	0.0
#define EKF10DOF_Q0_INIT 	1.0
#define EKF10DOF_QX_INIT 	0.0
#define EKF10DOF_QY_INIT 	0.0
#define EKF10DOF_BX_INIT 	0.0
#define EKF10DOF_BY_INIT 	0.0
#define EKF10DOF_BZ_INIT 	0.0

class EKF10DOF
{
	public:
		EKF10DOF();

		static void initialise();
		void iterate(const float p_us, const float v_ofs[2], const float w_imu[3], const float a_imu[3]);

		void getAltitudeEstimate();
		void getVelocityEstimate();
		void getQuaternionEstimate();
		void getStateEstimate();

		static void reset();

		bool isRunning,openLoop;

		static float altitudeEstimate,velocityEstimate[3],quaternionEstimate[3],accelerationEstimate[3];
		static uint16_t iterationTime;

	private:
		void populateSkewSymmetricMatrices(const float w_imu[3]);
		void populateJacobians(const float p_us, const float w_imu[3]);
		void calculateKalmanGain();
		void updateErrorCovariance();
		void updateEstimate(const float p_us, const float v_ofs[2], const float w_imu[3]);
		void projectStateAhead(const float w_imu[3],const float a_imu[3]);

		static float P[100],Q[81],R[9],A[100],C[30],L[90],K[30],H[100],E[3];
		float AP[100],At[100],APAt[100],LQ[90],Lt[90],Ct[30],CP[30],CPCt[9],
			  CPCtPlusR[9],invCPCtPlusR[9],PCt[30],KC[100],I10MinusKC[100],Ptemp[100],KE[10];
//		float LQLt[100];
		float Sq_w[9],Sv_v[9],Sv_w[9],Ap_v[9],Ap_q[9],Av_v[9],Av_q[9],Av_b[9],Aq_q[9],Aq_b[9],
			  Lv_w[9],Lv_a[9],Lq_w[9],Lb_b[9];
		float I3[9],I10[100];
		static float x_[10],x_hat[10],y[3],y_hat[3];
};

#endif /* EKF10DOF_H_ */
