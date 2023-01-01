#include "motorspeedcontrol.h"

double motorSpeedControl::Kp,
       motorSpeedControl::Kw,
       motorSpeedControl::reference,
       motorSpeedControl::motorSpeed[4],
       motorSpeedControl::motorSpeedNotched[2],
       motorSpeedControl::escCom[4],
       motorSpeedControl::escComMax,
       motorSpeedControl::batteryVoltage,
       motorSpeedControl::current[4];

bool motorSpeedControl::motorsArmed,
     motorSpeedControl::isClosedLoop,
     motorSpeedControl::torqueFeedForward,
     motorSpeedControl::dwEnabled,
     motorSpeedControl::isSaturated;

void motorSpeedControl::checkSaturation()
{
    if( (motorSpeedControl::escCom[0] >= MOTORSPEEDCONTROL_ESC_MAX) || (motorSpeedControl::escCom[1] >= MOTORSPEEDCONTROL_ESC_MAX) )
    {
        isSaturated = true;
    }
    else
    {
        isSaturated = false;
    }
}
