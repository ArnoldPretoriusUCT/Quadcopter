#ifndef SERIALRADIO_H
#define SERIALRADIO_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QtCore>
#include <QtCore/QDebug>
#include <QDateTime>
#include <QTimer>

#include "motorspeedcontrol.h"
#include "attitudecontrol.h"
#include "positioncontrol.h"

#define RADIO_SPEKTRUM_CH0        0b00001100
#define RADIO_SPEKTRUM_CH1        0b00000100
#define RADIO_SPEKTRUM_CH2        0b00001000
#define RADIO_SPEKTRUM_CH3        0b00010000
#define RADIO_SPEKTRUM_CH4 		  0b00000000
#define RADIO_SPEKTRUM_CH5 		  0b00010100

#define RADIO_VALUE_RANGE_CH0     516
#define RADIO_VALUE_RANGE_CH1     510
#define RADIO_VALUE_RANGE_CH2     510
#define RADIO_VALUE_RANGE_CH3     682
#define RADIO_VALUE_RANGE_CH4     820
#define RADIO_VALUE_RANGE_CH5     682

#define RADIO_WATCHDOG_THRESHOLD  25

#define RADIO_BAUD_RATE           115200
#define RADIO_BUFFER_SIZE         16

#define RADIO_PORT_NAME           "/dev/RADIO"

class serialRadio: public QObject
{
    Q_OBJECT
    public Q_SLOTS:
        void initialise();
        void getData();
        static void timeStampData();
        static void watchDog();

    public:
        bool newData;
        static bool isOpen,commsIssue;
        static unsigned char watchDogCount;
        quint16 channelValue[6],channelValueNom[6];
        static double controllerCommand[3];
        static double frequency,dT,dTMean,dTCount;

    private:
        void getControllerCommands();
        void getNominalChannelValues();
        int packetSize;
        QSerialPort *serialPort;
        QByteArray receiveBuffer;
        QTimer *watchDogTimer;
        static double receiveTime,receiveTime_past;
        bool isInitialised;

};

#endif // SERIALRADIO_H
