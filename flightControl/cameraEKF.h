  #ifndef cameraEKF_H
#define cameraEKF_H

#include <armadillo>
#include <iostream>
#include <QDateTime>
#include <QElapsedTimer>

#include "motioncapture.h"
#include "common.h"

using namespace arma;
using namespace std;

//initial error covariance diagonals
#define P11     pow(.01,2) //(m)
#define P22     pow(.01,2)
#define P33     pow(.01,2)
#define P44     pow(.00,2) //(m/s)
#define P55     pow(.00,2)
#define P66     pow(.00,2)
#define P77     pow(.005,2) //(rad/rad)
#define P88     pow(.005,2)
#define P99     pow(.005,2)
#define P1010   pow(.005,2)
//initial state estimate
#define PX      0.0
#define PY      0.0
#define PZ      0.0
#define Q0     1.0
#define QX     0.0
#define QY     0.0
#define QZ     0.0
//sample time
#define CAMERAEKF_DT      1.0e-3
//body-frame feature locations
#define B1_X    0.0
#define B1_Y    0.215
#define B1_Z    0.0
#define B2_X    0.0
#define B2_Y   -0.215
#define B2_Z    0.0
#define B3_X   -0.215
#define B3_Y    0.0
#define B3_Z    0.0
//standard deviations
#define EKF_SIG_VX   5 //(m/s/s)
#define EKF_SIG_VY   5
#define EKF_SIG_VZ   10
#define EKF_SIG_W   1.0e-3 //(rad/s)
#define EKF_SIG_W_CAMERA_DATA_ONLY 5e1 //(rad/s)
#define EKF_SIG_PX  1.8e-3 //(pixel/pixel)
#define EKF_SIG_PY  2.0e-3 //(pixel/pixel)

class cameraEKF: public QObject
{
    Q_OBJECT
    public:
        static void initialise();
        static void forwardPropagation();
        static void outputEstimate();
        static void outputSensitivity();
        static void innovation(vec p1, vec p2, vec p3, vec p4, vec p5);
        static void iterate(vec p1,vec p2,vec p3,vec p4,vec p5);
        static void timeStampData();

        static vec x;
        static mat P;
        static bool isActive,cameraDataOnly;
        static double dT, iterationTime;
    private:
        static vec x_;
        static mat Q,R,L,C,I;
        static mat R1,R2,R3,R4,R5;
        static vec T1,T2,T3,T4,T5;
        static vec b1,b2,b3;

        static vec p,v,w;
        static mat R_; //rotation matrix
        static double q0,q1,q2,q3,w1,w2,w3;
        static vec z1,z2,z3;
        static vec y1,y2,y3,y4,y5,y_;

};

#endif // cameraEKF_H
