#ifndef STATEMACHINE_H
#define STATEMACHINE_H


class stateMachine
{
    public:
        static bool calibrationMode,servoCentering;
        static bool heaveControl;
        static bool ledsOn;
        static bool resetMicro;
        static bool isWarning;
        static bool servoActive[4];

        static double thrust_0,thrust;
};

#endif // STATEMACHINE_H
