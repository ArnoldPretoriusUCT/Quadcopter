#ifndef PIXY_H
#define PIXY_H

#include <QtCore/QDebug>

#define PIXY_START_WORD            0xAA55
#define PIXY_START_WORD_CC         0xAA56
#define PIXY_START_WORDX           0x55AA

#define PIXY_RED_SIGNATURE         1
#define PIXY_BLUE_SIGNATURE        2
#define PIXY_GREEN_SIGNATURE       3
#define PIXY_WHITE_SIGNATURE       4

class object
{
    public:
    unsigned char signature;
    quint16 checkSum,pixelPosition[2],pixelHeight,pixelWidth;
};
class pixy
{
    public:
       object blob[4];
       quint8 objectsDetected;

       void getObjectInformation();
};


#endif // PIXY_H
