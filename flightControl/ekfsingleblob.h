#ifndef EKFSINGLEBLOB_H
#define EKFSINGLEBLOB_H

#include <armadillo>
#include <iostream>
#include <QDateTime>

#include "motioncapture.h"

using namespace arma;
using namespace std;

//initial error covariance diagonals
#define EKF_SINGLE_BLOB_P11     pow(.03,2) //(m)
#define EKF_SINGLE_BLOB_P22     pow(.03,2)
#define EKF_SINGLE_BLOB_P33     pow(.03,2)
#define EKF_SINGLE_BLOB_P44     pow(.00,2) //(m/s)
#define EKF_SINGLE_BLOB_P55     pow(.00,2)
#define EKF_SINGLE_BLOB_P66     pow(.00,2)
//initial state estimate
#define EKF_SINGLE_BLOB_PX      0.0
#define EKF_SINGLE_BLOB_PY      0.0
#define EKF_SINGLE_BLOB_PZ      0.0
#define EKF_SINGLE_BLOB_VX      0.0
#define EKF_SINGLE_BLOB_VY      0.0
#define EKF_SINGLE_BLOB_VZ      0.0
//standard deviations
#define EKF_SINGLE_BLOB_SIG_VX   10 //(m/s/s)
#define EKF_SINGLE_BLOB_SIG_VY   10
#define EKF_SINGLE_BLOB_SIG_VZ   10
#define EKF_SINGLE_BLOB_SIG_PX  3.3e-3 //(pixel/pixel)
#define EKF_SINGLE_BLOB_SIG_PY  2.0e-3 //(pixel/pixel)

class ekfSingleBlob: public QObject
{
    Q_OBJECT
    public:
        static void initialise();
        static void forwardPropagation();
        static void outputEstimate();
        static void outputSensitivity();
        static void innovation(vec p1, vec p2, vec p3, vec p4, vec p5);
        static void iterate(vec p1,vec p2,vec p3,vec p4,vec p5);

        static vec x;
        static mat P;
        static bool isActive;
        static double dT, iterationTime;
    private:
        static vec x_,y_;
        static mat Q,R,L,H,I,A;
        static mat R1,R2,R3,R4,R5;
        static vec T1,T2,T3,T4,T5;
};

#endif // EKFSINGLEBLOB_H
