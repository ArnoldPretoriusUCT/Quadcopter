#ifndef SERIALTEENSY_H
#define SERIALTEENSY_H

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QtCore/QDebug>
#include <QSignalMapper>
#include <QDateTime>
#include <QObject>

#define SERIAL_TEENSY_COM_PORT  "COM4"
#define SERIAL_TEENSY_BAUD_RATE 230400

class serialTeensy: public QObject
{
    Q_OBJECT

    public Q_SLOTS:
        void initialise(QString comPort, qint32 baudRate);
        void getData();
        void sendData(char data);
    public:
        QSerialPort *serialPort;
        char receiveBuffer[100];
        float runTime,motorSpeed;
};

#endif // SERIALTEENSY_H
