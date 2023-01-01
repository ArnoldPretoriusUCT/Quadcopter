#ifndef SERIALPIXY_H
#define SERIALPIXY_H

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QtCore/QDebug>
#include <QSignalMapper>
#include <QDateTime>
#include <QObject>
#include <QElapsedTimer>
#include <QTimer>

#include "common.h"
#include "statemachine.h"

#define SERIAL_PIXY_BAUD_RATE 460800
#define SERIAL_PIXY_COM_PORT0 "/dev/PIXY1"
#define SERIAL_PIXY_COM_PORT1 "/dev/PIXY2"
#define SERIAL_PIXY_COM_PORT2 "/dev/PIXY3"
#define SERIAL_PIXY_COM_PORT3 "/dev/PIXY4"
#define SERIAL_PIXY_COM_PORT4 "/dev/PIXY5"
//#define SERIAL_PIXY_COM_PORT5 "ttyUSB5"

#define PIXY_START_WORD            0xAA55
#define PIXY_START_WORD_CC         0xAA56
#define PIXY_START_WORDX           0x55AA

#define SERIAL_PIXY_WATCHDOG_THRESHOLD 4

class object
{
    public:
        unsigned char signature;
        quint16 checkSum,pixelPosition[2],pixelHeight,pixelWidth;
};

class serialPixy: public QObject
{
    Q_OBJECT

    public:
        double receiveTime,sendTime,dT;
        QSerialPort *serialPort;
//        char receiveBuffer[100];
        QByteArray receiveBuffer;
        char sendBuffer[5];
        bool newData;

        object blob[4];
        quint8 objectsDetected;

        static bool commsIssue;

    public slots:

        void initialise(QString comPort, qint32 baudRate);
        void reset(QString comPort, qint32 baudRate);
        void watchDog();

    public Q_SLOTS:
        void getData();
        void sendData();
        void getObjectInformation();

    private:
        void timeStampData();

        quint16 packetSize,watchDogCount;
        bool syncWordsFound;
        QTimer *watchDogTimer;
        double receiveTime_past;

};

#endif // SERIALPIXY_H

