#ifndef MOTORSPEEDCONTROL_H
#define MOTORSPEEDCONTROL_H

#define MOTORSPEEDCONTROL_KP 20
#define MOTORSPEEDCONTROL_KW 3

#define MOTORSPEEDCONTROL_ESC_MAX 3250

#define MOTORSPEEDCONTROL_PWM_MIN 13107
#define MOTORSPEEDCONTROL_NOMINAL_SETPOINT 600

class motorSpeedControl
{
    public:
        static void checkSaturation();

        static double Kp,Kw,reference,motorSpeed[4],motorSpeedNotched[2],escCom[4],escComMax,batteryVoltage,current[4];
        static bool motorsArmed,isClosedLoop,torqueFeedForward,dwEnabled,isSaturated;
};

#endif // MOTORSPEEDCONTROL_H
