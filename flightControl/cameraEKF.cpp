#include "cameraEKF.h"

vec cameraEKF::x,
    cameraEKF::x_;
mat cameraEKF::P(10,10);
bool cameraEKF::isActive,
         cameraEKF::cameraDataOnly;
double cameraEKF::dT;
double cameraEKF::iterationTime;

mat cameraEKF::Q,cameraEKF::R,cameraEKF::L,cameraEKF::C,cameraEKF::I,cameraEKF::R1,cameraEKF::R2,cameraEKF::R3,cameraEKF::R4,cameraEKF::R5;
vec cameraEKF::T1,cameraEKF::T2,cameraEKF::T3,cameraEKF::T4,cameraEKF::T5,cameraEKF::b1,cameraEKF::b2,cameraEKF::b3;

vec cameraEKF::p,cameraEKF::v,cameraEKF::w;
mat cameraEKF::R_;
double cameraEKF::q0,cameraEKF::q1,cameraEKF::q2,cameraEKF::q3,cameraEKF::w1,cameraEKF::w2,cameraEKF::w3;
vec cameraEKF::z1,cameraEKF::z2,cameraEKF::z3;
vec cameraEKF::y1,cameraEKF::y2,cameraEKF::y3,cameraEKF::y4,cameraEKF::y5,cameraEKF::y_;

void cameraEKF::initialise()
{
    x << PX << PY << PZ << 0 << 0 << 0 << Q0 << QX << QY << QZ << endr;
    dT = CAMERAEKF_DT;

    vec Pdiag;
    Pdiag << P11 << P22 << P33 << P44 << P55 << P66 << P77 << P88 << P99 << P1010 << endr;
    P = diagmat(Pdiag);
    I.eye(10,10);

    R1 = motionCapture::C1.cols(0,2);
    R2 = motionCapture::C2.cols(0,2);
    R3 = motionCapture::C3.cols(0,2);
    R4 = motionCapture::C4.cols(0,2);
    R5 = motionCapture::C5.cols(0,2);

    T1 = motionCapture::C1.col(3);
    T2 = motionCapture::C2.col(3);
    T3 = motionCapture::C3.col(3);
    T4 = motionCapture::C4.col(3);
    T5 = motionCapture::C5.col(3);

    b1 << B1_X << B1_Y << B1_Z << endr;
    b2 << B2_X << B2_Y << B2_Z << endr;
    b3 << B3_X << B3_Y << B3_Z << endr;

    Q.zeros(6,6);
    L.zeros(10,6);    
}

void cameraEKF::forwardPropagation()
{
    mat W(4,4);
    W <<   0                     << -imu::gyro[0] << -imu::gyro[1] << -imu::gyro[2] << endr
         << imu::gyro[0] <<            0             <<  imu::gyro[2] << -imu::gyro[1] << endr
         << imu::gyro[1] << -imu::gyro[2] <<            0             <<  imu::gyro[0] << endr
         << imu::gyro[2] <<  imu::gyro[1] << -imu::gyro[0] <<           0              << endr;

    //State transition matrix A
    mat A(10,10),Aq(4,4);
    vec Qdiag;

    if( cameraDataOnly )
    {
        Aq = eye(4,4);
        Qdiag << pow(EKF_SIG_VX,2) << pow(EKF_SIG_VY,2) << pow(EKF_SIG_VZ,2) << pow(EKF_SIG_W_CAMERA_DATA_ONLY,2) << pow(EKF_SIG_W_CAMERA_DATA_ONLY,2) << pow(EKF_SIG_W_CAMERA_DATA_ONLY,2) << endr;
    }
    else
    {
        Aq = eye(4,4) + dT/2*W;
        Qdiag << pow(EKF_SIG_VX,2) << pow(EKF_SIG_VY,2) << pow(EKF_SIG_VZ,2) << pow(EKF_SIG_W,2) << pow(EKF_SIG_W,2) << pow(EKF_SIG_W,2) << endr;
    }

    A.zeros(10,10);
    A.submat(0,0,2,2)     = eye(3,3);
    A.submat(0,3,2,5)     = dT*eye(3,3);
    A.submat(3,3,5,5)     = eye(3,3);
    A.submat(6,6,9,9)     = Aq;
    x_ = A*x;
    p = x_.rows(0,2);
    v = x_.rows(3,5);
    q0 = x_(6);
    q1 = x_(7);
    q2 = x_(8);
    q3 = x_(9);

    //Input noise jacobian
    mat Lw(4,3);
    Lw << -q1 << -q2 << -q3 << endr
          <<  q0 << -q3 <<  q2 << endr
          <<  q3 <<  q0 << -q1 << endr
          << -q2 <<  q1 <<  q0 << endr;
    Lw = dT/2*Lw;
    L.submat(3,0,5,2) = dT*eye(3,3);
    L.submat(6,3,9,5) = Lw;
    //A priori error covariance P        
    Q = diagmat(Qdiag);
    P = A*P*trans(A) + L*Q*trans(L);
    //rotation matrix
    R_ << 1-2*(q2*q2+q3*q3) << 2*(q1*q2-q0*q3)   <<  2*(q1*q3+q0*q2)  << endr
          <<  2*(q1*q2+q0*q3)  << 1-2*(q1*q1+q3*q3) << 2*(q2*q3-q0*q1)   << endr
          <<  2*(q1*q3-q0*q2)  <<  2*(q2*q3+q0*q1)  << 1-2*(q1*q1+q2*q2) << endr;
}

void cameraEKF::outputEstimate()
{
    mat Rtemp;
    vec Ttemp;

    z1 = p + R_*b1; //inertial-frame feature1 estimate
    z2 = p + R_*b2; //inertial-frame feature2 estimate
    z3 = p + R_*b3; //inertial-frame feature3 estimate
    //y1
    Rtemp = R1.rows(0,1);
    Ttemp = T1.rows(0,1);
    y1.zeros(6);
    y1.rows(0,1) = Rtemp*z1 + Ttemp;
    y1.rows(2,3) = Rtemp*z2 + Ttemp;
    y1.rows(4,5) = Rtemp*z3 + Ttemp;
    //y2
    Rtemp = R2.rows(0,1);
    Ttemp = T2.rows(0,1);
    y2.zeros(6);
    y2.rows(0,1) = Rtemp*z1 + Ttemp;
    y2.rows(2,3) = Rtemp*z2 + Ttemp;
    y2.rows(4,5) = Rtemp*z3 + Ttemp;
    //y3
    Rtemp = R3.rows(0,1);
    Ttemp = T3.rows(0,1);
    y3.zeros(6);
    y3.rows(0,1) = Rtemp*z1 + Ttemp;
    y3.rows(2,3) = Rtemp*z2 + Ttemp;
    y3.rows(4,5) = Rtemp*z3 + Ttemp;
    //y4
    Rtemp = R4.rows(0,1);
    Ttemp = T4.rows(0,1);
    y4.zeros(6);
    y4.rows(0,1) = Rtemp*z1 + Ttemp;
    y4.rows(2,3) = Rtemp*z2 + Ttemp;
    y4.rows(4,5) = Rtemp*z3 + Ttemp;
    //y5
    Rtemp = R5.rows(0,1);
    Ttemp = T5.rows(0,1);
    y5.zeros(6);
    y5.rows(0,1) = Rtemp*z1 + Ttemp;
    y5.rows(2,3) = Rtemp*z2 + Ttemp;
    y5.rows(4,5) = Rtemp*z3 + Ttemp;
    //y_
    y_.zeros(30);
    y_.rows(0,5)   = y1;
    y_.rows(6,11)  = y2;
    y_.rows(12,17) = y3;
    y_.rows(18,23) = y4;
    y_.rows(24,29) = y5;
}

void cameraEKF::outputSensitivity()
{
    double R11,R12,R13,R21,R22,R23;
    mat Cp(2,3),Cq(2,4);
    mat C11(2,10),C12(2,10),C13(2,10),C21(2,10),C22(2,10),C23(2,10),C31(2,10),C32(2,10),C33(2,10),C41(2,10),C42(2,10),C43(2,10),C51(2,10),C52(2,10),C53(2,10);

    //CAMERA 1
    R11 = R1(0,0);
    R12 = R1(0,1);
    R13 = R1(0,2);
    R21 = R1(1,0);
    R22 = R1(1,1);
    R23 = R1(1,2);
    //C11
    Cp = R1.rows(0,1);
    Cq << (2*R13*q1) - (2*R11*q3) << (2*R11*q2) - (4*R12*q1) + (2*R13*q0) << (2*R11*q1) + (2*R13*q3) << (2*R13*q2) - (4*R12*q3) - (2*R11*q0) << endr
          << (2*R23*q1) - (2*R21*q3) << (2*R21*q2) - (4*R22*q1) + (2*R23*q0) << (2*R21*q1) + (2*R23*q3) << (2*R23*q2) - (4*R22*q3) - (2*R21*q0) << endr;
    Cq = B1_Y*Cq;
    C11.cols(0,2) = Cp;
    C11.cols(3,5) = zeros(2,3);
    C11.cols(6,9) = Cq;
    //C12
    Cp = R1.rows(0,1);
    Cq << (2*R13*q1) - (2*R11*q3) << (2*R11*q2) - (4*R12*q1) + (2*R13*q0) << (2*R11*q1) + (2*R13*q3) << (2*R13*q2) - (4*R12*q3) - (2*R11*q0) << endr
          << (2*R23*q1) - (2*R21*q3) << (2*R21*q2) - (4*R22*q1) + (2*R23*q0) << (2*R21*q1) + (2*R23*q3) << (2*R23*q2) - (4*R22*q3) - (2*R21*q0) << endr;
    Cq = B2_Y*Cq;
    C12.cols(0,2) = Cp;
    C12.cols(3,5) = zeros(2,3);
    C12.cols(6,9) = Cq;
    //C13
    Cp = R1.rows(0,1);
    Cq << (2*R12*q3) - (2*R13*q2) << (2*R12*q2) + (2*R13*q3) << (2*R12*q1) - (4*R11*q2) - (2*R13*q0) << (2*R12*q0) - (4*R11*q3) + (2*R13*q1) << endr
          << (2*R22*q3) - (2*R23*q2) << (2*R22*q2) + (2*R23*q3) << (2*R22*q1) - (4*R21*q2) - (2*R23*q0) << (2*R22*q0) - (4*R21*q3) + (2*R23*q1) << endr;
    Cq = B3_X*Cq;
    C13.cols(0,2) = Cp;
    C13.cols(3,5) = zeros(2,3);
    C13.cols(6,9) = Cq;

    //CAMERA 2
    R11 = R2(0,0);
    R12 = R2(0,1);
    R13 = R2(0,2);
    R21 = R2(1,0);
    R22 = R2(1,1);
    R23 = R2(1,2);
    //C21
    Cp = R2.rows(0,1);
    Cq << (2*R13*q1) - (2*R11*q3) << (2*R11*q2) - (4*R12*q1) + (2*R13*q0) << (2*R11*q1) + (2*R13*q3) << (2*R13*q2) - (4*R12*q3) - (2*R11*q0) << endr
          << (2*R23*q1) - (2*R21*q3) << (2*R21*q2) - (4*R22*q1) + (2*R23*q0) << (2*R21*q1) + (2*R23*q3) << (2*R23*q2) - (4*R22*q3) - (2*R21*q0) << endr;
    Cq = B1_Y*Cq;
    C21.cols(0,2) = Cp;
    C21.cols(3,5) = zeros(2,3);
    C21.cols(6,9) = Cq;
    //C22
    Cp = R2.rows(0,1);
    Cq << (2*R13*q1) - (2*R11*q3) << (2*R11*q2) - (4*R12*q1) + (2*R13*q0) << (2*R11*q1) + (2*R13*q3) << (2*R13*q2) - (4*R12*q3) - (2*R11*q0) << endr
          << (2*R23*q1) - (2*R21*q3) << (2*R21*q2) - (4*R22*q1) + (2*R23*q0) << (2*R21*q1) + (2*R23*q3) << (2*R23*q2) - (4*R22*q3) - (2*R21*q0) << endr;
    Cq = B2_Y*Cq;
    C22.cols(0,2) = Cp;
    C22.cols(3,5) = zeros(2,3);
    C22.cols(6,9) = Cq;
    //C23
    Cp = R2.rows(0,1);
    Cq << (2*R12*q3) - (2*R13*q2) << (2*R12*q2) + (2*R13*q3) << (2*R12*q1) - (4*R11*q2) - (2*R13*q0) << (2*R12*q0) - (4*R11*q3) + (2*R13*q1) << endr
          << (2*R22*q3) - (2*R23*q2) << (2*R22*q2) + (2*R23*q3) << (2*R22*q1) - (4*R21*q2) - (2*R23*q0) << (2*R22*q0) - (4*R21*q3) + (2*R23*q1) << endr;
    Cq = B3_X*Cq;
    C23.cols(0,2) = Cp;
    C23.cols(3,5) = zeros(2,3);
    C23.cols(6,9) = Cq;

    //CAMERA 3
    R11 = R3(0,0);
    R12 = R3(0,1);
    R13 = R3(0,2);
    R21 = R3(1,0);
    R22 = R3(1,1);
    R23 = R3(1,2);
    //C31
    Cp = R3.rows(0,1);
    Cq << (2*R13*q1) - (2*R11*q3) << (2*R11*q2) - (4*R12*q1) + (2*R13*q0) << (2*R11*q1) + (2*R13*q3) << (2*R13*q2) - (4*R12*q3) - (2*R11*q0) << endr
          << (2*R23*q1) - (2*R21*q3) << (2*R21*q2) - (4*R22*q1) + (2*R23*q0) << (2*R21*q1) + (2*R23*q3) << (2*R23*q2) - (4*R22*q3) - (2*R21*q0) << endr;
    Cq = B1_Y*Cq;
    C31.cols(0,2) = Cp;
    C31.cols(3,5) = zeros(2,3);
    C31.cols(6,9) = Cq;
    //C32
    Cp = R3.rows(0,1);
    Cq << (2*R13*q1) - (2*R11*q3) << (2*R11*q2) - (4*R12*q1) + (2*R13*q0) << (2*R11*q1) + (2*R13*q3) << (2*R13*q2) - (4*R12*q3) - (2*R11*q0) << endr
          << (2*R23*q1) - (2*R21*q3) << (2*R21*q2) - (4*R22*q1) + (2*R23*q0) << (2*R21*q1) + (2*R23*q3) << (2*R23*q2) - (4*R22*q3) - (2*R21*q0) << endr;
    Cq = B2_Y*Cq;
    C32.cols(0,2) = Cp;
    C32.cols(3,5) = zeros(2,3);
    C32.cols(6,9) = Cq;
    //C33
    Cp = R3.rows(0,1);
    Cq << (2*R12*q3) - (2*R13*q2) << (2*R12*q2) + (2*R13*q3) << (2*R12*q1) - (4*R11*q2) - (2*R13*q0) << (2*R12*q0) - (4*R11*q3) + (2*R13*q1) << endr
          << (2*R22*q3) - (2*R23*q2) << (2*R22*q2) + (2*R23*q3) << (2*R22*q1) - (4*R21*q2) - (2*R23*q0) << (2*R22*q0) - (4*R21*q3) + (2*R23*q1) << endr;
    Cq = B3_X*Cq;
    C33.cols(0,2) = Cp;
    C33.cols(3,5) = zeros(2,3);
    C33.cols(6,9) = Cq;

    //CAMERA 4
    R11 = R4(0,0);
    R12 = R4(0,1);
    R13 = R4(0,2);
    R21 = R4(1,0);
    R22 = R4(1,1);
    R23 = R4(1,2);
    //C41
    Cp = R4.rows(0,1);
    Cq << (2*R13*q1) - (2*R11*q3) << (2*R11*q2) - (4*R12*q1) + (2*R13*q0) << (2*R11*q1) + (2*R13*q3) << (2*R13*q2) - (4*R12*q3) - (2*R11*q0) << endr
          << (2*R23*q1) - (2*R21*q3) << (2*R21*q2) - (4*R22*q1) + (2*R23*q0) << (2*R21*q1) + (2*R23*q3) << (2*R23*q2) - (4*R22*q3) - (2*R21*q0) << endr;
    Cq = B1_Y*Cq;
    C41.cols(0,2) = Cp;
    C41.cols(3,5) = zeros(2,3);
    C41.cols(6,9) = Cq;
    //C42
    Cp = R4.rows(0,1);
    Cq << (2*R13*q1) - (2*R11*q3) << (2*R11*q2) - (4*R12*q1) + (2*R13*q0) << (2*R11*q1) + (2*R13*q3) << (2*R13*q2) - (4*R12*q3) - (2*R11*q0) << endr
          << (2*R23*q1) - (2*R21*q3) << (2*R21*q2) - (4*R22*q1) + (2*R23*q0) << (2*R21*q1) + (2*R23*q3) << (2*R23*q2) - (4*R22*q3) - (2*R21*q0) << endr;
    Cq = B2_Y*Cq;
    C42.cols(0,2) = Cp;
    C42.cols(3,5) = zeros(2,3);
    C42.cols(6,9) = Cq;
    //C43
    Cp = R4.rows(0,1);
    Cq << (2*R12*q3) - (2*R13*q2) << (2*R12*q2) + (2*R13*q3) << (2*R12*q1) - (4*R11*q2) - (2*R13*q0) << (2*R12*q0) - (4*R11*q3) + (2*R13*q1) << endr
          << (2*R22*q3) - (2*R23*q2) << (2*R22*q2) + (2*R23*q3) << (2*R22*q1) - (4*R21*q2) - (2*R23*q0) << (2*R22*q0) - (4*R21*q3) + (2*R23*q1) << endr;
    Cq = B3_X*Cq;
    C43.cols(0,2) = Cp;
    C43.cols(3,5) = zeros(2,3);
    C43.cols(6,9) = Cq;

    //CAMERA 5
    R11 = R5(0,0);
    R12 = R5(0,1);
    R13 = R5(0,2);
    R21 = R5(1,0);
    R22 = R5(1,1);
    R23 = R5(1,2);
    //C51
    Cp = R5.rows(0,1);
    Cq << (2*R13*q1) - (2*R11*q3) << (2*R11*q2) - (4*R12*q1) + (2*R13*q0) << (2*R11*q1) + (2*R13*q3) << (2*R13*q2) - (4*R12*q3) - (2*R11*q0) << endr
          << (2*R23*q1) - (2*R21*q3) << (2*R21*q2) - (4*R22*q1) + (2*R23*q0) << (2*R21*q1) + (2*R23*q3) << (2*R23*q2) - (4*R22*q3) - (2*R21*q0) << endr;
    Cq = B1_Y*Cq;
    C51.cols(0,2) = Cp;
    C51.cols(3,5) = zeros(2,3);
    C51.cols(6,9) = Cq;
    //C52
    Cp = R5.rows(0,1);
    Cq << (2*R13*q1) - (2*R11*q3) << (2*R11*q2) - (4*R12*q1) + (2*R13*q0) << (2*R11*q1) + (2*R13*q3) << (2*R13*q2) - (4*R12*q3) - (2*R11*q0) << endr
          << (2*R23*q1) - (2*R21*q3) << (2*R21*q2) - (4*R22*q1) + (2*R23*q0) << (2*R21*q1) + (2*R23*q3) << (2*R23*q2) - (4*R22*q3) - (2*R21*q0) << endr;
    Cq = B2_Y*Cq;
    C52.cols(0,2) = Cp;
    C52.cols(3,5) = zeros(2,3);
    C52.cols(6,9) = Cq;
    //C53
    Cp = R5.rows(0,1);
    Cq << (2*R12*q3) - (2*R13*q2) << (2*R12*q2) + (2*R13*q3) << (2*R12*q1) - (4*R11*q2) - (2*R13*q0) << (2*R12*q0) - (4*R11*q3) + (2*R13*q1) << endr
          << (2*R22*q3) - (2*R23*q2) << (2*R22*q2) + (2*R23*q3) << (2*R22*q1) - (4*R21*q2) - (2*R23*q0) << (2*R22*q0) - (4*R21*q3) + (2*R23*q1) << endr;
    Cq = B3_X*Cq;
    C53.cols(0,2) = Cp;
    C53.cols(3,5) = zeros(2,3);
    C53.cols(6,9) = Cq;
    //C
    C.zeros(30,10);
    C.rows(0,1)   = C11;
    C.rows(2,3)   = C12;
    C.rows(4,5)   = C13;
    C.rows(6,7)   = C21;
    C.rows(8,9)   = C22;
    C.rows(10,11) = C23;
    C.rows(12,13) = C31;
    C.rows(14,15) = C32;
    C.rows(16,17) = C33;
    C.rows(18,19) = C41;
    C.rows(20,21) = C42;
    C.rows(22,23) = C43;
    C.rows(24,25) = C51;
    C.rows(26,27) = C52;
    C.rows(28,29) = C53;
}

void cameraEKF::innovation(vec p1, vec p2, vec p3, vec p4, vec p5)
{
    //feature depth estimates
    vec w11 = R1.row(2)*z1 + T1(2);
    vec w12 = R1.row(2)*z2 + T1(2);
    vec w13 = R1.row(2)*z3 + T1(2);
    vec w21 = R2.row(2)*z1 + T2(2);
    vec w22 = R2.row(2)*z2 + T2(2);
    vec w23 = R2.row(2)*z3 + T2(2);
    vec w31 = R3.row(2)*z1 + T3(2);
    vec w32 = R3.row(2)*z2 + T3(2);
    vec w33 = R3.row(2)*z3 + T3(2);
    vec w41 = R4.row(2)*z1 + T4(2);
    vec w42 = R4.row(2)*z2 + T4(2);
    vec w43 = R4.row(2)*z3 + T4(2);
    vec w51 = R5.row(2)*z1 + T5(2);
    vec w52 = R5.row(2)*z2 + T5(2);
    vec w53 = R5.row(2)*z3 + T5(2);

    vec Mdiag(30);
    Mdiag << w11(0) << w11(0) << w12(0) << w12(0) << w13(0) << w13(0) << w21(0) << w21(0) << w22(0) << w22(0) << w23(0) << w23(0) << w31(0) << w31(0) << w32(0) << w32(0) << w33(0) << w33(0) << w41(0) << w41(0) << w42(0) << w42(0) << w43(0) << w43(0) << w51(0) << w51(0) << w52(0) << w52(0) << w53(0) << w53(0) << endr;
    mat M = diagmat(Mdiag);

    vec p(30);
    p.rows(0,5)   = p1;
    p.rows(6,11)  = p2;
    p.rows(12,17) = p3;
    p.rows(18,23) = p4;
    p.rows(24,29) = p5;
    //output measurement equation
    vec y = M*p;

    R.zeros(30,30);
    for(int i=0;i<15;i++)
    {
        R(2*i,2*i) = pow(EKF_SIG_PX,2);
        R(2*i+1,2*i+1) = pow(EKF_SIG_PY,2);
    }
    for(int i=0;i<30;i++)
    {
        if(p(i) == 10)
        {
            R(i,i) = 1e5;   //penalise invalid measurements
            y(i) = y_(i);
        }
    }
    //Kalman gain
    mat K = P*trans(C) * inv( C*P*trans(C)+M*R*trans(M) );
    //update estimate
    x = x_ + K*(y - y_);
    x.rows(6,9) = x.rows(6,9)/norm(x.rows(6,9));
    //update error covariance
    P = (I - K*C)*P;
}

void cameraEKF::iterate(vec p1,vec p2,vec p3,vec p4,vec p5)
{
    timeStampData();
    if(cameraEKF::isActive)
    {        
        forwardPropagation();
        outputEstimate();
        outputSensitivity();
        innovation(p1,p2,p3,p4,p5);
    }

}

void cameraEKF::timeStampData()
{
    static double iterationTime_past = 0;
    iterationTime = ((double)elapsedTimer.nsecsElapsed())*1e-6;
    dT = (iterationTime-iterationTime_past)*1e-3;
    iterationTime_past = iterationTime;
}
