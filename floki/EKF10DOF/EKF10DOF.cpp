/*
 * EKF10DOF.cpp
 *
 *  Created on: 04 Apr 2016
 *      Author: Arnold
 */
#include "EKF10DOF.h"

float EKF10DOF::altitudeEstimate,
	  EKF10DOF::velocityEstimate[3],
	  EKF10DOF::quaternionEstimate[3],
	  EKF10DOF::accelerationEstimate[3];

uint16_t EKF10DOF::iterationTime;

float EKF10DOF::P[100],EKF10DOF::Q[81],EKF10DOF::R[9],EKF10DOF::A[100],EKF10DOF::C[30],EKF10DOF::L[90],EKF10DOF::K[30],EKF10DOF::H[100],EKF10DOF::E[3];
float EKF10DOF::x_[10],EKF10DOF::x_hat[10],EKF10DOF::y[3],EKF10DOF::y_hat[3];

EKF10DOF::EKF10DOF()
{
	isRunning = openLoop = false;
	altitudeEstimate = EKF10DOF_PZ_INIT;
	memset(&velocityEstimate[0],0,4*3);
	memset(&quaternionEstimate[0],0,4*3);
	memset(&accelerationEstimate[0],0,4*3);

	memset(&x_[0],0,4*10);

	memset(&Q[0],0,4*81);
	Q[0]  = EKF10DOF_Q11;
	Q[10] = EKF10DOF_Q22;
	Q[20] = EKF10DOF_Q33;
	Q[30] = EKF10DOF_Q44;
	Q[40] = EKF10DOF_Q55;
	Q[50] = EKF10DOF_Q66;
	Q[60] = EKF10DOF_Q77;
	Q[70] = EKF10DOF_Q88;
	Q[80] = EKF10DOF_Q99;

	memset(&R[0],0,4*9);
	R[0] = EKF10DOF_R11;
	R[4] = EKF10DOF_R22;
	R[8] = EKF10DOF_R33;

	memset(&A[0],0,4*100);
	A[0] = 1.0; //Ap_p
	A[17] = A[28] = A[39] = -dT; //Av_b
	A[77] =	A[88] = A[99] = 1.0; //Ab_b

	memset(&Av_q[0],0,4*9);
	memset(&Aq_q[0],0,4*9);
	memset(&Aq_b[0],0,4*9);

	memset(&L[0],0,4*90);
	L[12] = L[22] = L[32] = -dT; //Lv_a
	L[69] = L[79] = L[89] =  dT; //Lb_b

	memset(&Lv_w[0],0,4*9);
	memset(&Lq_w[0],0,4*9);

	memset(&Sq_w[0],0,4*9);
	memset(&Sv_w[0],0,4*9);
	memset(&Sv_v[0],0,4*9);

	memset(&C[0],0,4*30);
	C[20] = 1.0;

	memset(&H[0],0,4*100);
	H[0] = H[11] = H[22] = H[33] = H[77] = H[88] = H[99] = 1.0;

	memset(&I3[0],0,4*9);
	I3[0] = I3[4] = I3[8] = 1.0;
	memset(&I10[0],0,4*100);
	I10[0] = I10[11] = I10[22] = I10[33] = I10[44] = I10[55] = I10[66] = I10[77] = I10[88] = I10[99] = 1.0;
}

void EKF10DOF::populateSkewSymmetricMatrices(const float w_imu[3])
{
							Sq_w[1]  = -w_imu[0];	Sq_w[2]  = -w_imu[1];
	Sq_w[3] = w_imu[0];								Sq_w[5]  =  w_imu[2];
	Sq_w[6] = w_imu[1];		Sq_w[7]  = -w_imu[2];

							Sv_w[1] = -w_imu[2];	Sv_w[2] =  w_imu[1];
	Sv_w[3] =  w_imu[2];   							Sv_w[5] = -w_imu[0];
	Sv_w[6] = -w_imu[1];	Sv_w[7] =  w_imu[0];

							Sv_v[1] = -x_hat[3];	Sv_v[2] =  x_hat[2];
	Sv_v[3] =  x_hat[3];    			     		Sv_v[5] = -x_hat[1];
	Sv_v[6] = -x_hat[2];	Sv_v[7] =  x_hat[1];
}

void EKF10DOF::populateJacobians(const float p_us, const float w_imu[3])
{
	populateSkewSymmetricMatrices(w_imu);

	Ap_v[0] =  dT*(1 - 2*x_hat[6]*x_hat[6]);	Ap_v[1] = dT*2*x_hat[5]*x_hat[6]; 			Ap_v[2] =  dT*2*x_hat[4]*x_hat[6];
	Ap_v[3] =  dT*2*x_hat[5]*x_hat[6]; 			Ap_v[4] = dT*(1-2*x_hat[5]*x_hat[5]);		Ap_v[5] = -dT*2*x_hat[4]*x_hat[5];
	Ap_v[6] = -dT*2*x_hat[4]*x_hat[6]; 			Ap_v[7] = dT*2*x_hat[4]*x_hat[5];    		Ap_v[8] =  dT*(1 - 2*(x_hat[5]*x_hat[5] + x_hat[6]*x_hat[6])); //(3x3)

	Ap_q[0] = dT*2*x_hat[6]*x_hat[3];							Ap_q[1] = dT*2*x_hat[6]*x_hat[2];												Ap_q[2] =  dT*(2*x_hat[4]*x_hat[3] + 2*x_hat[5]*x_hat[2] - 4*x_hat[6]*x_hat[1]);
	Ap_q[3] = -dT*2*x_hat[5]*x_hat[3];							Ap_q[4] = dT*(2*x_hat[6]*x_hat[1] - 4*x_hat[5]*x_hat[2] - 2*x_hat[4]*x_hat[3]);	Ap_q[5] =  dT*2*x_hat[5]*x_hat[1];
	Ap_q[6] = dT*(2*x_hat[5]*x_hat[2] - 2*x_hat[6]*x_hat[1]);	Ap_q[7] = dT*(2*x_hat[4]*x_hat[2] - 4*x_hat[5]*x_hat[3]);						Ap_q[8] = -dT*(4*x_hat[6]*x_hat[3] + 2*x_hat[4]*x_hat[1]);

    //	Av_v = I - dT*wx_w;
	float dtWx_w[9] = {0};
	scaleArray(Sv_w,9,dT,&dtWx_w[0]);
	Matrix.Subtract(I3,dtWx_w,3,3,&Av_v[0]);

	Av_q[0] =  g*dT*2*x_hat[6];										Av_q[2] = g*dT*2*x_hat[4];
	Av_q[3] = -g*dT*2*x_hat[5]; 	Av_q[4] = -g*dT*2*x_hat[4];
	Av_q[6] = -g*dT*2*x_hat[4];		Av_q[7] =  g*dT*2*x_hat[5];		Av_q[8] = g*dT*2*x_hat[6];

//	Aq_q = I+dT/2*Wx; %(3x3)
	float dtWx_q[9] = {0};
	scaleArray(Sq_w,9,dT/2,&dtWx_q[0]);
	Matrix.Add(I3,dtWx_q,3,3,&Aq_q[0]);

	A[1]  = Ap_v[6];	A[2]  = Ap_v[7];	A[3]  = Ap_v[8];		A[4]  = Ap_q[6];	A[5]  = Ap_q[7];    A[6]  = Ap_q[8];

	A[11] = Av_v[0];	A[12] = Av_v[1];	A[13] = Av_v[2];		A[14] = Av_q[0];	A[15] = Av_q[1];	A[16] = Av_q[2];
	A[21] = Av_v[3];	A[22] = Av_v[4];	A[23] = Av_v[5];		A[24] = Av_q[3];	A[25] = Av_q[4];	A[26] = Av_q[5];
	A[31] = Av_v[6];	A[32] = Av_v[7];	A[33] = Av_v[8];		A[34] = Av_q[6];	A[35] = Av_q[7];	A[36] = Av_q[8];

	A[44] = Aq_q[0];	A[45] = Aq_q[1];	A[46] = Aq_q[2];
	A[54] = Aq_q[3];	A[55] = Aq_q[4];	A[56] = Aq_q[5];
	A[64] = Aq_q[6];	A[65] = Aq_q[7];	A[66] = Aq_q[8];

//	memcpy(&Lv_w[0],&Av_b[0],4*9);
	scaleArray(Sv_v,9,-dT,&Lv_w[0]);

//	memcpy(&Lq_w[0],&Aq_b[0],4*9);
	Lq_w[0] =  dT/2*x_hat[5];	Lq_w[1] =  dT/2*x_hat[6];
	Lq_w[3] = -dT/2*x_hat[4];    							Lq_w[5] = -dT/2*x_hat[6];
								Lq_w[7] = -dT/2*x_hat[4];	Lq_w[8] =  dT/2*x_hat[5];

	L[9]  = Lv_w[0]; 	L[10] = Lv_w[1]; 	L[11] = Lv_w[2]; 		//L[12] = Lv_a[0]; L[13] = Lv_a[1]; L[14] = Lv_a[2]; //CONSTANT
	L[18] = Lv_w[3];	L[19] = Lv_w[4]; 	L[20] = Lv_w[5]; 		//L[21] = Lv_a[3]; L[22] = Lv_a[4]; L[23] = Lv_a[5];
	L[27] = Lv_w[6];	L[28] = Lv_w[7]; 	L[29] = Lv_w[8]; 		//L[30] = Lv_a[6]; L[31] = Lv_a[7]; L[32] = Lv_a[8];

	L[36] = Lq_w[0];	L[37] = Lq_w[1]; 	L[38] = Lq_w[2];
	L[45] = Lq_w[3];	L[46] = Lq_w[4]; 	L[47] = Lq_w[5];
	L[54] = Lq_w[6];	L[55] = Lq_w[7]; 	L[56] = Lq_w[8];

//	L[69] = Lb_b[0];	L[70] = Lb_b[1]; 	L[71] = Lb_b[2]; //CONSTANT
//	L[78] = Lb_b[3];	L[79] = Lb_b[4]; 	L[80] = Lb_b[5];
//	L[87] = Lb_b[6];	L[88] = Lb_b[7]; 	L[89] = Lb_b[8];

	C[0]  = ADNS3080_FOCAL_LENGTH*x_[1]/(x_[0]*x_[0])*(2*x_[4]*x_[4]-1);	C[1] =  -ADNS3080_FOCAL_LENGTH/x_[0]*(2*x_[4]*x_[4]-1); 	C[4] =  -4*ADNS3080_FOCAL_LENGTH*x_[4]*x_[1]/x_[0];
	C[10] = ADNS3080_FOCAL_LENGTH*x_[2]/(x_[0]*x_[0])*(2*x_[4]*x_[4]-1);	C[12] = -ADNS3080_FOCAL_LENGTH/x_[0]*(2*x_[4]*x_[4]-1); 	C[14] = -4*ADNS3080_FOCAL_LENGTH*x_[4]*x_[2]/x_[0];
//																								C[24] = 4*EKF10DOF_COM_2_USM*x_[4];
}

void EKF10DOF::calculateKalmanGain()
{
	//Error covariance
	//APA'
	Matrix.Multiply(A,P,10,10,10,&AP[0]);
	Matrix.Transpose(A,10,10,&At[0]);
	Matrix.Multiply(AP,At,10,10,10,&APAt[0]);

	//LQL'
//	Matrix.Multiply(L,Q,10,9,9,&LQ[0]);
//	Matrix.Transpose(L,10,9,&Lt[0]);
//	Matrix.Multiply(LQ,Lt,10,9,10,&LQLt[0]);
	float LQLt[] = { 0,                                                                   0,                                                                   0,                                                                   0,                                                         0,                                                         0,                                                           0,                 0,                 0,                 0,
					 0, EKF10DOF_Q11*dT*dT*x_hat[2]*x_hat[2] + EKF10DOF_Q11*dT*dT*x_hat[3]*x_hat[3] + EKF10DOF_Q44*dT*dT,                                            -EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[2],                                            -EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[3],                               (EKF10DOF_Q11*dT*dT*x_hat[3]*x_hat[6])/2,                               (EKF10DOF_Q11*dT*dT*x_hat[2]*x_hat[6])/2, - (EKF10DOF_Q11*dT*dT*x_hat[2]*x_hat[5])/2 - (EKF10DOF_Q11*dT*dT*x_hat[3]*x_hat[4])/2,                 0,                 0,                 0,
					 0,                                            -EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[2], EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[1] + EKF10DOF_Q11*dT*dT*x_hat[3]*x_hat[3] + EKF10DOF_Q44*dT*dT,                                            -EKF10DOF_Q11*dT*dT*x_hat[2]*x_hat[3],                              -(EKF10DOF_Q11*dT*dT*x_hat[3]*x_hat[5])/2, (EKF10DOF_Q11*dT*dT*x_hat[3]*x_hat[4])/2 - (EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[6])/2,                                 (EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[5])/2,                 0,                 0,                 0,
					 0,                                            -EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[3],                                            -EKF10DOF_Q11*dT*dT*x_hat[2]*x_hat[3], EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[1] + EKF10DOF_Q11*dT*dT*x_hat[2]*x_hat[2] + EKF10DOF_Q44*dT*dT, (EKF10DOF_Q11*dT*dT*x_hat[2]*x_hat[5])/2 - (EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[6])/2,                              -(EKF10DOF_Q11*dT*dT*x_hat[2]*x_hat[4])/2,                                 (EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[4])/2,                 0,                 0,                 0,
					 0,                                         (EKF10DOF_Q11*dT*dT*x_hat[3]*x_hat[6])/2,                                        -(EKF10DOF_Q11*dT*dT*x_hat[3]*x_hat[5])/2,           (EKF10DOF_Q11*dT*dT*x_hat[2]*x_hat[5])/2 - (EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[6])/2,   (EKF10DOF_Q11*dT*dT*x_hat[5]*x_hat[5])/4 + (EKF10DOF_Q11*dT*dT*x_hat[6]*x_hat[6])/4,                              -(EKF10DOF_Q11*dT*dT*x_hat[4]*x_hat[5])/4,                                -(EKF10DOF_Q11*dT*dT*x_hat[4]*x_hat[6])/4,                 0,                 0,                 0,
					 0,                                         (EKF10DOF_Q11*dT*dT*x_hat[2]*x_hat[6])/2,           (EKF10DOF_Q11*dT*dT*x_hat[3]*x_hat[4])/2 - (EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[6])/2,                                        -(EKF10DOF_Q11*dT*dT*x_hat[2]*x_hat[4])/2,                              -(EKF10DOF_Q11*dT*dT*x_hat[4]*x_hat[5])/4,   (EKF10DOF_Q11*dT*dT*x_hat[4]*x_hat[4])/4 + (EKF10DOF_Q11*dT*dT*x_hat[6]*x_hat[6])/4,                                -(EKF10DOF_Q11*dT*dT*x_hat[5]*x_hat[6])/4,                 0,                 0,                 0,
					 0,         - (EKF10DOF_Q11*dT*dT*x_hat[2]*x_hat[5])/2 - (EKF10DOF_Q11*dT*dT*x_hat[3]*x_hat[4])/2,                                         (EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[5])/2,                                         (EKF10DOF_Q11*dT*dT*x_hat[1]*x_hat[4])/2,                              -(EKF10DOF_Q11*dT*dT*x_hat[4]*x_hat[6])/4,                              -(EKF10DOF_Q11*dT*dT*x_hat[5]*x_hat[6])/4,     (EKF10DOF_Q11*dT*dT*x_hat[4]*x_hat[4])/4 + (EKF10DOF_Q11*dT*dT*x_hat[5]*x_hat[5])/4,                 0,                 0,                 0,
					 0,                                                                   0,                                                                   0,                                                                   0,                                                         0,                                                         0,                                                           0, EKF10DOF_Q77*dT*dT,                 0,                 0,
					 0,                                                                   0,                                                                   0,                                                                   0,                                                         0,                                                         0,                                                           0,                 0, EKF10DOF_Q77*dT*dT,                 0,
					 0,                                                                   0,                                                                   0,                                                                   0,                                                         0,                                                         0,                                                           0,                 0,                 0, EKF10DOF_Q77*dT*dT};


	//P
	Matrix.Add(APAt,LQLt,10,10,&P[0]);

	//Kalman gain
	//CPC'
	Matrix.Multiply(C,P,3,10,10,&CP[0]);
	Matrix.Transpose(C,3,10,&Ct[0]);
	Matrix.Multiply(CP,Ct,3,10,3,&CPCt[0]);

	//1/(CPC'+R)
	Matrix.Add(CPCt,R,3,3,&CPCtPlusR[0]);
	invert3by3Matrix(CPCtPlusR,&invCPCtPlusR[0]);

	//P_C'
	Matrix.Multiply(P,Ct,10,10,3,&PCt[0]);

	//K = PC'/(CPC'+MRM')
	Matrix.Multiply(PCt,invCPCtPlusR,10,3,3,&K[0]);
}

void EKF10DOF::updateErrorCovariance()
{
	//I-KC
	Matrix.Multiply(K,C,10,3,10,&KC[0]);
	Matrix.Subtract(I10,KC,10,10,&I10MinusKC[0]);

	//P = (I-K*C)*P_
	Matrix.Multiply(I10MinusKC,P,10,10,10,&Ptemp[0]);
	memcpy(&P[0],&Ptemp[0],4*100);
}

void EKF10DOF::updateEstimate(const float p_us, const float v_ofs[2], const float w_imu[3])
{
	//y_hat = C*x_hat
	y_hat[0] = -ADNS3080_FOCAL_LENGTH*x_[1]/x_[0]*(2*x_[4]*x_[4]-1);
	y_hat[1] = -ADNS3080_FOCAL_LENGTH*x_[2]/x_[0]*(2*x_[4]*x_[4]-1);
	y_hat[2] =  x_[0];// - EKF10DOF_COM_2_USM*(2*x_[4]*x_[4]-1);

	//y
	y[0] = v_ofs[0]-ADNS3080_FOCAL_LENGTH*w_imu[1];
	y[1] = v_ofs[1]+ADNS3080_FOCAL_LENGTH*w_imu[0];
	y[2] = p_us;

	//Innovation residual
	Matrix.Subtract(y,y_hat,3,1,&E[0]);
	//Update estimate
	Matrix.Multiply(K,E,10,3,1,&KE[0]);

	Matrix.Add(x_,KE,10,1,&x_hat[0]);

	float quat_norm = sqrtf(x_hat[4]*x_hat[4]+x_hat[5]*x_hat[5]+x_hat[6]*x_hat[6]);
	x_hat[4] = x_hat[4]/quat_norm;
	x_hat[5] = x_hat[5]/quat_norm;
	x_hat[6] = x_hat[6]/quat_norm;
}

void EKF10DOF::projectStateAhead(const float w_imu[3],const float a_imu[3])
{
							Sq_w[1]  = -w_imu[0];	Sq_w[2]  = -w_imu[1];
	Sq_w[3] = w_imu[0];								Sq_w[5]  =  w_imu[2];
	Sq_w[6] = w_imu[1];		Sq_w[7]  = -w_imu[2];

	float dtWx_q[9] = {0};
	scaleArray(Sq_w,9,dT/2,&dtWx_q[0]);
	Matrix.Add(I3,dtWx_q,3,3,&Aq_q[0]);

	Ap_v[6] = -dT*2*x_hat[4]*x_hat[6];		Ap_v[7] = dT*2*x_hat[4]*x_hat[5];    	Ap_v[8] =  dT*(1 - 2*(x_hat[5]*x_hat[5] + x_hat[6]*x_hat[6]));

	float gb[3];
	gb[0] = -g*2*x_hat[4]*x_hat[6];
	gb[1] =  g*2*x_hat[4]*x_hat[5];
	gb[2] =  g*(x_hat[4]*x_hat[4]-x_hat[5]*x_hat[5]-x_hat[6]*x_hat[6]);

	float v_est[3];
	v_est[0] = x_hat[1];
	v_est[1] = x_hat[2];
	v_est[2] = x_hat[3];

	float wxv[3] = {0};
	cross(w_imu,v_est, &wxv[0]);

	accelerationEstimate[0] = g*a_imu[0] - gb[0] - wxv[0] - x_hat[7];
	accelerationEstimate[1] = g*a_imu[1] - gb[1] - wxv[1] - x_hat[8];
	accelerationEstimate[2] = g*a_imu[2] - gb[2] - wxv[2] - x_hat[9];

	float dTa[10] = {0};
	dTa[1] = dT*accelerationEstimate[0];
	dTa[2] = dT*accelerationEstimate[1];
	dTa[3] = dT*accelerationEstimate[2];

	H[1]  = Ap_v[6];		H[2]  = Ap_v[7];		H[3]  = Ap_v[8];

	H[44] = Aq_q[0];		H[45] = Aq_q[1];		H[46] = Aq_q[2];
	H[54] = Aq_q[3];		H[55] = Aq_q[4];		H[56] = Aq_q[5];
	H[64] = Aq_q[6];		H[65] = Aq_q[7];		H[66] = Aq_q[8];

	float Hx_hat[10] = {0};
	Matrix.Multiply(H,x_hat,10,10,1,&Hx_hat[0]);
	Matrix.Add(Hx_hat,dTa,10,1, &x_[0]);
}

void EKF10DOF::initialise()
{
	x_[0] = EKF10DOF_PZ_INIT;
	x_[1] = EKF10DOF_VX_INIT;
	x_[2] = EKF10DOF_VY_INIT;
	x_[3] = EKF10DOF_VZ_INIT;
	x_[4] = EKF10DOF_Q0_INIT;
	x_[5] = EKF10DOF_QX_INIT;
	x_[6] = EKF10DOF_QY_INIT;
	x_[7] = EKF10DOF_BX_INIT;
	x_[8] = EKF10DOF_BY_INIT;
	x_[9] = EKF10DOF_BZ_INIT;

	memcpy(&x_hat[0],&x_[0],4*10);

	memset(&P[0],0,4*100);
	P[0]  = EKF10DOF_P11;
	P[11] = EKF10DOF_P22;
	P[22] = EKF10DOF_P33;
	P[33] = EKF10DOF_P44;
	P[44] = EKF10DOF_P55;
	P[55] = EKF10DOF_P66;
	P[66] = EKF10DOF_P77;
	P[77] = EKF10DOF_P88;
	P[88] = EKF10DOF_P99;
	P[99] = EKF10DOF_P1010;
}

void EKF10DOF::iterate(const float p_us, const float v_ofs[2], const float w_imu[3], const float a_imu[3])
{
//	uint16_t millis_start = millis();
	populateJacobians(p_us, w_imu);
	calculateKalmanGain();
	updateErrorCovariance();
	updateEstimate(p_us, v_ofs, w_imu);
	projectStateAhead(w_imu, a_imu);
//	iterationTime = millis()-millis_start;
//	Serial.println(iterationTime);

	isRunning = true;
}

void EKF10DOF::reset()
{
	initialise();
}

void EKF10DOF::getAltitudeEstimate()
{
	altitudeEstimate = x_hat[0];
}

void EKF10DOF::getVelocityEstimate()
{
	memcpy(&velocityEstimate[0],&x_hat[1],4*3);
}

void EKF10DOF::getQuaternionEstimate()
{
	memcpy(&quaternionEstimate[0],&x_hat[4],4*3);
}

void EKF10DOF::getStateEstimate()
{
	getAltitudeEstimate();
	getVelocityEstimate();
	getQuaternionEstimate();
}
