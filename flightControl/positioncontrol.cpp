#include "positioncontrol.h"

double positionControl::heaveCommand;

void positionControl::initialise()
{
    heaveCommand = POSITIONCONTROL_THRUST_MIN;
}

