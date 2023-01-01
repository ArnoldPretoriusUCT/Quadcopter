#ifndef MOTIONCAPTURE_H
#define MOTIONCAPTURE_H

#include <QObject>
#include <armadillo>
#include <iostream>
#include "common.h"
#include "cameraEKF.h"
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <string>
#include <QDateTime>
#include <QFile>
#include <QTimer>
#include <QElapsedTimer>
#include <QThread>
#include <QtCore>
#include <QtCore/QDebug>

#include "serialPixy.h"
#include "cameraEKF.h"
#include "ekfsingleblob.h"
#include "imu.h"
#include "motorspeedcontrol.h"
#include "attitudecontrol.h"
#include "datalogger.h"

#define EKF_MULTI_BLOB 0
#define EKF_SINGLE_BLOB 1

using namespace arma;
using namespace std;

class motionCapture: public QObject
{
    Q_OBJECT
    public Q_SLOTS:
        void initialise();
        void getData(vec sig,vec fDet,cube Bp,cube fSizes);
        void backProjectPoints(vec b1,vec b2,vec b3);
        void normalizeFeatures();
        void correspondenceMatching(vec pos,vec quat);
        void featurePositionEstimates(vec pos,vec quat);
        void validFeatures(vec m1, vec m2, vec m3, vec m4, vec m5);
        void initialisePixies();
        void updateData();
        void initialiseTimers(int t_msec);

        static void watchDog();

    public:
        vec matchFeatures(mat B,mat Bx,double fDetected,mat fSizes);
        mat undistort(mat B,vec kc);
        vec orderFeatures(mat B,vec m);

        static mat C1,C2,C3,C4,C5;
        vec p1,p2,p3,p4,p5;
        static vec fValid,camValidMean,camValidCount,xyzValidMean,valid;
        static double xyzValidCount;
        static bool isFullyObservable,newData[5];
        static bool ekfType,isCritical;

        static vec fDet,fDetMean,sig,dT,dTMean,meanCount,receiveTime;
        static cube Bp,fSizes;
        static double startTime;
        static quint8 watchDogCount;

        serialPixy pixy[5];

    private:        
        mat K1,K2,K3,K4,K5;
        mat B1,B2,B3,B4,B5;
        mat B1x,B2x,B3x,B4x,B5x;
        vec kc1,kc2,kc3,kc4,kc5;
        vec m1,m2,m3,m4,m5;
        mat S;
        vec b1,b2,b3;
        double m_array1[3],m_array2[3],m_array3[3],m_array4[3],m_array5[3];

        mat fSizes1,fSizes2,fSizes3,fSizes4,fSizes5;

        vec signature, fDetected;

        QElapsedTimer timer;
//        static FILE *f1;
//        static char log[100];
};

#endif // MOTIONCAPTURE_H
