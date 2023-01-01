#include "ekfsingleblob.h"

vec ekfSingleBlob::x,
    ekfSingleBlob::x_,
    ekfSingleBlob::y_;
mat ekfSingleBlob::P(6,6);
bool ekfSingleBlob::isActive;
double ekfSingleBlob::dT;

mat ekfSingleBlob::A,
    ekfSingleBlob::H,
    ekfSingleBlob::Q,
    ekfSingleBlob::R,
    ekfSingleBlob::L,
    ekfSingleBlob::I,
    ekfSingleBlob::R1,ekfSingleBlob::R2,ekfSingleBlob::R3,ekfSingleBlob::R4,ekfSingleBlob::R5;
vec ekfSingleBlob::T1,ekfSingleBlob::T2,ekfSingleBlob::T3,ekfSingleBlob::T4,ekfSingleBlob::T5;

void ekfSingleBlob::initialise()
{
    x << EKF_SINGLE_BLOB_PX << EKF_SINGLE_BLOB_PY << EKF_SINGLE_BLOB_PZ << EKF_SINGLE_BLOB_VX << EKF_SINGLE_BLOB_VY << EKF_SINGLE_BLOB_VZ << endr;
    dT = CAMERAEKF_DT;

    vec Pdiag;
    Pdiag << EKF_SINGLE_BLOB_P11 << EKF_SINGLE_BLOB_P22 << EKF_SINGLE_BLOB_P33 << EKF_SINGLE_BLOB_P44 << EKF_SINGLE_BLOB_P55 << EKF_SINGLE_BLOB_P66 << endr;
    P = diagmat(Pdiag);
    I.eye(6,6);

    vec Qdiag;
    Qdiag << pow(EKF_SINGLE_BLOB_SIG_VX,2) << pow(EKF_SINGLE_BLOB_SIG_VY,2) << pow(EKF_SINGLE_BLOB_SIG_VZ,2) << endr;
    Q.zeros(3,3);
    Q = diagmat(Qdiag);

    L.zeros(6,3);
    L << 0  << 0  << 0  << endr
      << 0  << 0  << 0  << endr
      << 0  << 0  << 0  << endr
      << dT << 0  << 0  << endr
      << 0  << dT << 0  << endr
      << 0  << 0  << dT << endr;

    A << 1 << 0 << 0 << dT << 0  << 0  << endr
      << 0 << 1 << 0 << 0  << dT << 0  << endr
      << 0 << 0 << 1 << 0  << 0  << dT << endr
      << 0 << 0 << 0 << 1  << 0  << 0  << endr
      << 0 << 0 << 0 << 0  << 1  << 0  << endr
      << 0 << 0 << 0 << 0  << 0  << 1  << endr;
}

void ekfSingleBlob::forwardPropagation()
{
    x_ = A*x;
    P = A*P*trans(A)+L*Q*trans(L);
}

void ekfSingleBlob::outputEstimate()
{
    vec p,y1_,y2_,y3_,y4_,y5_;
    p << x_(0) << x_(1) << x_(2) << 1;
    y1_ = motionCapture::C1.rows(0,1)/as_scalar(motionCapture::C1.row(2)*p)*p;
    y2_ = motionCapture::C2.rows(0,1)/as_scalar(motionCapture::C2.row(2)*p)*p;
    y3_ = motionCapture::C3.rows(0,1)/as_scalar(motionCapture::C3.row(2)*p)*p;
    y4_ = motionCapture::C4.rows(0,1)/as_scalar(motionCapture::C4.row(2)*p)*p;
    y5_ = motionCapture::C5.rows(0,1)/as_scalar(motionCapture::C5.row(2)*p)*p;

    y_.zeros(10,1);
    y_.rows(0,1) = y1_;
    y_.rows(2,3) = y2_;
    y_.rows(4,5) = y3_;
    y_.rows(6,7) = y4_;
    y_.rows(8,9) = y5_;
}

void ekfSingleBlob::outputSensitivity()
{
    mat H1,H2,H3,H4,H5,C;
    C.zeros(4,5);
    C.submat(1,1,3,4) = motionCapture::C1;
    H1 << (C(1,1)*C(3,4) - C(1,4)*C(3,1) + C(1,1)*C(3,2)*x_(1) - C(1,2)*C(3,1)*x_(1) + C(1,1)*C(3,3)*x_(2) - C(1,3)*C(3,1)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(1,2)*C(3,4) - C(1,4)*C(3,2) - C(1,1)*C(3,2)*x_(0) + C(1,2)*C(3,1)*x_(0) + C(1,2)*C(3,3)*x_(2) - C(1,3)*C(3,2)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(1,3)*C(3,4) - C(1,4)*C(3,3) - C(1,1)*C(3,3)*x_(0) + C(1,3)*C(3,1)*x_(0) - C(1,2)*C(3,3)*x_(1) + C(1,3)*C(3,2)*x_(1))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << endr
       << (C(2,1)*C(3,4) - C(2,4)*C(3,1) + C(2,1)*C(3,2)*x_(1) - C(2,2)*C(3,1)*x_(1) + C(2,1)*C(3,3)*x_(2) - C(2,3)*C(3,1)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(2,2)*C(3,4) - C(2,4)*C(3,2) - C(2,1)*C(3,2)*x_(0) + C(2,2)*C(3,1)*x_(0) + C(2,2)*C(3,3)*x_(2) - C(2,3)*C(3,2)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(2,3)*C(3,4) - C(2,4)*C(3,3) - C(2,1)*C(3,3)*x_(0) + C(2,3)*C(3,1)*x_(0) - C(2,2)*C(3,3)*x_(1) + C(2,3)*C(3,2)*x_(1))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << endr;

    C.submat(1,1,3,4) = motionCapture::C2;
    H2 << (C(1,1)*C(3,4) - C(1,4)*C(3,1) + C(1,1)*C(3,2)*x_(1) - C(1,2)*C(3,1)*x_(1) + C(1,1)*C(3,3)*x_(2) - C(1,3)*C(3,1)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(1,2)*C(3,4) - C(1,4)*C(3,2) - C(1,1)*C(3,2)*x_(0) + C(1,2)*C(3,1)*x_(0) + C(1,2)*C(3,3)*x_(2) - C(1,3)*C(3,2)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(1,3)*C(3,4) - C(1,4)*C(3,3) - C(1,1)*C(3,3)*x_(0) + C(1,3)*C(3,1)*x_(0) - C(1,2)*C(3,3)*x_(1) + C(1,3)*C(3,2)*x_(1))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << endr
       << (C(2,1)*C(3,4) - C(2,4)*C(3,1) + C(2,1)*C(3,2)*x_(1) - C(2,2)*C(3,1)*x_(1) + C(2,1)*C(3,3)*x_(2) - C(2,3)*C(3,1)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(2,2)*C(3,4) - C(2,4)*C(3,2) - C(2,1)*C(3,2)*x_(0) + C(2,2)*C(3,1)*x_(0) + C(2,2)*C(3,3)*x_(2) - C(2,3)*C(3,2)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(2,3)*C(3,4) - C(2,4)*C(3,3) - C(2,1)*C(3,3)*x_(0) + C(2,3)*C(3,1)*x_(0) - C(2,2)*C(3,3)*x_(1) + C(2,3)*C(3,2)*x_(1))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << endr;

    C.submat(1,1,3,4) = motionCapture::C3;
    H3 << (C(1,1)*C(3,4) - C(1,4)*C(3,1) + C(1,1)*C(3,2)*x_(1) - C(1,2)*C(3,1)*x_(1) + C(1,1)*C(3,3)*x_(2) - C(1,3)*C(3,1)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(1,2)*C(3,4) - C(1,4)*C(3,2) - C(1,1)*C(3,2)*x_(0) + C(1,2)*C(3,1)*x_(0) + C(1,2)*C(3,3)*x_(2) - C(1,3)*C(3,2)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(1,3)*C(3,4) - C(1,4)*C(3,3) - C(1,1)*C(3,3)*x_(0) + C(1,3)*C(3,1)*x_(0) - C(1,2)*C(3,3)*x_(1) + C(1,3)*C(3,2)*x_(1))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << endr
       << (C(2,1)*C(3,4) - C(2,4)*C(3,1) + C(2,1)*C(3,2)*x_(1) - C(2,2)*C(3,1)*x_(1) + C(2,1)*C(3,3)*x_(2) - C(2,3)*C(3,1)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(2,2)*C(3,4) - C(2,4)*C(3,2) - C(2,1)*C(3,2)*x_(0) + C(2,2)*C(3,1)*x_(0) + C(2,2)*C(3,3)*x_(2) - C(2,3)*C(3,2)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(2,3)*C(3,4) - C(2,4)*C(3,3) - C(2,1)*C(3,3)*x_(0) + C(2,3)*C(3,1)*x_(0) - C(2,2)*C(3,3)*x_(1) + C(2,3)*C(3,2)*x_(1))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << endr;

    C.submat(1,1,3,4) = motionCapture::C4;
    H4 << (C(1,1)*C(3,4) - C(1,4)*C(3,1) + C(1,1)*C(3,2)*x_(1) - C(1,2)*C(3,1)*x_(1) + C(1,1)*C(3,3)*x_(2) - C(1,3)*C(3,1)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(1,2)*C(3,4) - C(1,4)*C(3,2) - C(1,1)*C(3,2)*x_(0) + C(1,2)*C(3,1)*x_(0) + C(1,2)*C(3,3)*x_(2) - C(1,3)*C(3,2)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(1,3)*C(3,4) - C(1,4)*C(3,3) - C(1,1)*C(3,3)*x_(0) + C(1,3)*C(3,1)*x_(0) - C(1,2)*C(3,3)*x_(1) + C(1,3)*C(3,2)*x_(1))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << endr
       << (C(2,1)*C(3,4) - C(2,4)*C(3,1) + C(2,1)*C(3,2)*x_(1) - C(2,2)*C(3,1)*x_(1) + C(2,1)*C(3,3)*x_(2) - C(2,3)*C(3,1)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(2,2)*C(3,4) - C(2,4)*C(3,2) - C(2,1)*C(3,2)*x_(0) + C(2,2)*C(3,1)*x_(0) + C(2,2)*C(3,3)*x_(2) - C(2,3)*C(3,2)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(2,3)*C(3,4) - C(2,4)*C(3,3) - C(2,1)*C(3,3)*x_(0) + C(2,3)*C(3,1)*x_(0) - C(2,2)*C(3,3)*x_(1) + C(2,3)*C(3,2)*x_(1))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << endr;

    C.submat(1,1,3,4) = motionCapture::C5;
    H5 << (C(1,1)*C(3,4) - C(1,4)*C(3,1) + C(1,1)*C(3,2)*x_(1) - C(1,2)*C(3,1)*x_(1) + C(1,1)*C(3,3)*x_(2) - C(1,3)*C(3,1)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(1,2)*C(3,4) - C(1,4)*C(3,2) - C(1,1)*C(3,2)*x_(0) + C(1,2)*C(3,1)*x_(0) + C(1,2)*C(3,3)*x_(2) - C(1,3)*C(3,2)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(1,3)*C(3,4) - C(1,4)*C(3,3) - C(1,1)*C(3,3)*x_(0) + C(1,3)*C(3,1)*x_(0) - C(1,2)*C(3,3)*x_(1) + C(1,3)*C(3,2)*x_(1))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << endr
       << (C(2,1)*C(3,4) - C(2,4)*C(3,1) + C(2,1)*C(3,2)*x_(1) - C(2,2)*C(3,1)*x_(1) + C(2,1)*C(3,3)*x_(2) - C(2,3)*C(3,1)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(2,2)*C(3,4) - C(2,4)*C(3,2) - C(2,1)*C(3,2)*x_(0) + C(2,2)*C(3,1)*x_(0) + C(2,2)*C(3,3)*x_(2) - C(2,3)*C(3,2)*x_(2))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << (C(2,3)*C(3,4) - C(2,4)*C(3,3) - C(2,1)*C(3,3)*x_(0) + C(2,3)*C(3,1)*x_(0) - C(2,2)*C(3,3)*x_(1) + C(2,3)*C(3,2)*x_(1))/pow(C(3,4) + C(3,1)*x_(0) + C(3,2)*x_(1) + C(3,3)*x_(2),2) << endr;

    H.zeros(10,6);
    H.submat(0,0,1,2) = H1;
    H.submat(2,0,3,2) = H2;
    H.submat(4,0,5,2) = H3;
    H.submat(6,0,7,2) = H4;
    H.submat(8,0,9,2) = H5;
}

void ekfSingleBlob::innovation(vec p1, vec p2, vec p3, vec p4, vec p5)
{
    vec y(10);
    y.rows(0,1) = p1;
    y.rows(2,3) = p2;
    y.rows(4,5) = p3;
    y.rows(6,7) = p4;
    y.rows(8,9) = p5;

    vec Rdiag;
    Rdiag << pow(EKF_SIG_PX,2) << pow(EKF_SIG_PY,2) << pow(EKF_SIG_PX,2) << pow(EKF_SIG_PY,2) << pow(EKF_SIG_PX,2) << pow(EKF_SIG_PY,2) << pow(EKF_SIG_PX,2) << pow(EKF_SIG_PY,2) << pow(EKF_SIG_PX,2) << pow(EKF_SIG_PY,2) << endr;
    R = diagmat(Rdiag);
    for(int i=0;i<5;i++)
    {
        if( (motionCapture::newData[i] == 0) || (motionCapture::fDet(i) == 0) )
        {
            R(2*i,2*i) = R(2*i+1,2*i+1) = 1e5; //discourage using invalid output measurements in update
        }
    }

    mat K = P*trans(H)*inv(H*P*trans(H)+R);    

    x = x_ + K*(y-y_);
    P = (I-K*H)*P;
}

void ekfSingleBlob::iterate(vec p1,vec p2,vec p3,vec p4,vec p5)
{
    if(ekfSingleBlob::isActive)
    {
        forwardPropagation();
        outputEstimate();
        outputSensitivity();
        innovation(p1,p2,p3,p4,p5);
    }
}
