#ifndef SERIALWIXEL_H
#define SERIALWIXEL_H

#include <armadillo>
#include <iostream>

using namespace arma;
using namespace std;

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QtCore>
#include <QtCore/QDebug>
#include <QApplication>
#include <QSignalMapper>
#include <QDateTime>
#include <QObject>
#include <QTimer>
#include <QElapsedTimer>
#include <QThread>

#include "cameraEKF.h"
#include "common.h"
#include "statemachine.h"
#include "attitudecontrol.h"
#include "motorspeedcontrol.h"
#include "imu.h"
#include "serialradio.h"
#include "positioncontrol.h"
#include "calibrationmode.h"
#include "motioncapture.h"

#define SERIAL_WIXEL_COM_PORT_RX                    "/dev/WIXEL_RX"
#define SERIAL_WIXEL_COM_PORT_TX                    "/dev/WIXEL_TX"
#define SERIAL_WIXEL_BAUD_RATE                          200000
#define SERIAL_WIXEL_SEND_BUFFER_SIZE           18
#define SERIAL_WIXEL_RECEIVE_BUFFER_SIZE     27
#define SERIAL_WIXEL_DT_MSEC                               10

#define SERIAL_WIXEL_HEAVE_SF                        2000
#define SERIAL_WIXEL_COMMAND_SF                1800
#define SERIAL_WIXEL_QUATERNION_SF           32767
#define SERIAL_WIXEL_GYRO_SF                        (32.8*180/M_PI)
#define SERIAL_WIXEL_ACCEL_SF                        16384.0
#define SERIAL_WIXEL_MOTOR_SPEED_SF        3.92
#define SERIAL_WIXEL_MOTOR_ESC_SF            51.4
#define SERIAL_WIXEL_SERVO_SF                       (ATTITUDECONTROL_SERVODELTA_MAX/127.0)
#define SERIAL_WIXEL_V_BAT_SF                        (12.4876/255.0)
#define SERIAL_WIXEL_MOTOR_CURRENT_SF  (1/25.5)

class serialWixel: public QSerialPort
{
    Q_OBJECT
    public:
        explicit serialWixel(QObject *parent = nullptr):QSerialPort(parent) {}

        static double receiveTime,sendTime,startTime;
        static double dT,dTMean,dTCount,frequency;
        QSerialPort *serialPortTx,*serialPortRx;
        QSerialPort *serialPort;
    //        char receiveBuffer[SERIAL_XBEE_RECEIVE_BUFFER_SIZE];
        QByteArray receiveBuffer,receiveBuffer_past;
        char sendBuffer[SERIAL_WIXEL_SEND_BUFFER_SIZE];

        bool newDataReceived,newDataSent;

        static bool commsIssue;

        static double masterServoCommand;
        static quint8 packetID;

    public Q_SLOTS:
        void initialise();
        void getData();
        void sendData();
        void getOnboardInformation();

        void watchDog();


    private:
        void timeStampData();
        int packetSize;        
        static unsigned char watchDogCount;
        double receiveTime_past,sendTime_past;
        QTimer *sendTimer;

//        int sPortTx;
};

#endif // SERIALWIXEL_H
