#include "attitudecontrol.h"

double attitudeControl::Kp[3],
       attitudeControl::Kw[3],
       attitudeControl::loopFrequency,
       attitudeControl::servoDelta[4],
       attitudeControl::servoDeltaMax,
       attitudeControl::thrustScaleFactor;
vec    attitudeControl::quaternion,
       attitudeControl::quaternionReference,
       attitudeControl::angleReference,
       attitudeControl::inputAction,
       attitudeControl::angleError,
       attitudeControl::C,
       attitudeControl::thrustEstimate;
bool   attitudeControl::isClosedLoop;

void attitudeControl::initialise()
{
    inputAction << 0 << 0 << endr;
    quaternionReference << 1 << 0 << 0 << 0 << endr;
    angleReference << 0 << 0 << 0 << endr;
    quaternion << 1 << 0 << 0 << 0 << endr;
    thrustEstimate << 0 << 0 << 0 << 0 << endr;

    servoDeltaMax = 1250;
    thrustScaleFactor=0;
    C << ATTITUDECONTROL_C1 << ATTITUDECONTROL_C2 << ATTITUDECONTROL_C3 << endr;
}

void attitudeControl::iterate()
{
    getThrustEstimates();
    getQuaternionReference();
    getQuaternionError();
    getInputAction();
}

void attitudeControl::getQuaternionReference()
{
    if( serialRadio::isOpen )
    {
        angleReference << serialRadio::controllerCommand[1]*ATTITUDECONTROL_ALPHA_MAX
                                       << serialRadio::controllerCommand[2]*ATTITUDECONTROL_ALPHA_MAX
                                       << 0
                                       << endr;        //intrinsic euler
    }
    else
    {
        angleReference << 0 << 0 << 0 << endr;
    }

    quaternionReference= euler2quaternion(angleReference);
}

void attitudeControl::getQuaternionError()
{
//    quaternion << cameraEKF::x(6) << cameraEKF::x(7) << cameraEKF::x(8) << cameraEKF::x(9) << endr;
    vec quaternionError = quatMultiply(quatConjugate(quaternion),quaternionReference);

    mat Re = quat2rotationMatrix(quaternionError);
    vec z_b,z_d;
    z_b << 0 << 0 << 1 << endr;
    z_d = Re*z_b;
    vec et = cross(z_b,z_d);
    double theta = asin( norm(et) );

    if( theta == 0 )
    {
        angleError << 0 << 0 << 0 << endr;
    }
    else
    {
        angleError = ( theta/sin(theta) )*et;  \

    }
}

void attitudeControl::getInputAction()
{
    inputAction(0) = ATTITUDECONTROL_PHI_K*angleError(0);
    inputAction(1) = ATTITUDECONTROL_THETA_K*angleError(1);

//    inputAction(0) = 10*angleReference(0);
//    inputAction(1) = 10*angleReference(1);
}

void attitudeControl::getThrustEstimates()
{
    double Kphi = 0.005968310365946;
    double tau = Kphi*motorSpeedControl::current[2];
    vec C;
    C << 0.000000001440873 <<  -0.000000177255461 <<  0.000049419383409 <<  0.004923621107686 << endr;
    double a = C(0);
    double b = C(1);
    double c = C(2);
    double d = C(3)-tau;
   thrustEstimate = pow(pow(pow(d/(2*a) + pow(b,3)/(27*pow(a,3)) - (b*c)/(6*pow(a,2)),2) + pow(- pow(b,2)/(9*pow(a,2)) + c/(3*a),3),0.5)- pow(b,3)/(27*pow(a,3)) - d/(2*a) + (b*c)/(6*pow(a,2)),1.0/3) - b/(3*a) - (- pow(b,2)/(9*pow(a,2)) + c/(3*a))/pow(pow(pow(d/(2*a) + pow(b,3)/(27*pow(a,3)) - (b*c)/(6*pow(a,2)),2) + pow(c/(3*a) - pow(b,2)/(9*pow(a,2)),3),.5) - pow(b,3)/(27*pow(a,3)) - d/(2*a) + (b*c)/(6*pow(a,2)),1.0/3);

}
