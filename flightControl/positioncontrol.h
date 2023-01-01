#ifndef POSITIONCONTROL_H
#define POSITIONCONTROL_H

#define POSITIONCONTROL_THRUST_MAX 20
#define POSITIONCONTROL_THRUST_MIN 3

class positionControl
{
    public:
        void initialise();
        static double heaveCommand;
};

#endif // POSITIONCONTROL_H
