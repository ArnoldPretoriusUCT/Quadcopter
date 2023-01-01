#ifndef ATTITUDECONTROL_H
#define ATTITUDECONTROL_H

#include "serialradio.h"
#include "cameraEKF.h"
#include "motorspeedcontrol.h"
#include "common.h"
#include <armadillo>
#include <iostream>

using namespace arma;
using namespace std;

#define ATTITUDECONTROL_PHI_K               7.0
#define ATTITUDECONTROL_THETA_K         7.0
#define ATTITUDECONTROL_PSI_K                0.0
#define ATTITUDECONTROL_ALPHA_MAX  (45*M_PI/180)

#define ATTITUDECONTROL_SERVODELTA_MAX 3500
#define ATTITUDECONTROL_C1 0.000027331682075//0.000014925665465//0.000011339508599
#define ATTITUDECONTROL_C2 0.000299539536580//0.002977892721223//0.003488139680153
#define ATTITUDECONTROL_C3 4.420364738635507//4.109645956569902//4.086044666313149

class attitudeControl
{
    public:
        static double Kp[3],
                      Kw[3],
                      loopFrequency,
                      servoDelta[4],
                      servoDeltaMax,
                      thrustScaleFactor;

        static vec    quaternion,
                      quaternionReference,
                      angleReference,
                      inputAction,
                      angleError,
                       C,
                       thrustEstimate;

        static void initialise();
        static void iterate();
        static void getThrustEstimates();

        static bool isClosedLoop;

    private:
        static void getQuaternionReference();
        static void getQuaternionError();
        static void getInputAction();
};

#endif // ATTITUDECONTROL_H
