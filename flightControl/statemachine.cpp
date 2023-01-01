#include "statemachine.h"

bool stateMachine::calibrationMode,
     stateMachine::servoCentering,
     stateMachine::heaveControl,
     stateMachine::ledsOn,
     stateMachine::resetMicro,
     stateMachine::isWarning,
     stateMachine::servoActive[4];

double stateMachine::thrust_0,
              stateMachine::thrust;

