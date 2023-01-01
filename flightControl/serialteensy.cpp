#include "serialteensy.h"

void serialTeensy::initialise(QString comPort, qint32 baudRate)
{
    serialPort = new QSerialPort();

    serialPort->setPortName(comPort);
    serialPort->open(QIODevice::ReadWrite);
    serialPort->setBaudRate(baudRate,QSerialPort::AllDirections);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    QObject::connect( serialPort, SIGNAL(readyRead()), this, SLOT( getData() ) );
    serialPort->clear(QSerialPort::AllDirections);
}

void serialTeensy::getData()
{
    serialPort->read(&receiveBuffer[0],7);
    serialPort->clear();

    runTime =  ( ((quint32)receiveBuffer[0]) << 24) | ( ((quint32)receiveBuffer[1]) << 16 ) | ( ((quint16)receiveBuffer[2]) << 8 ) | ( (quint8)receiveBuffer[3] ) ;
    runTime = (float)runTime/100;
    float ch0Count = (float)( ( (quint16)receiveBuffer[4] << 8) | ( (quint8)receiveBuffer[5] ) );
    quint8 pwmCommand = receiveBuffer[6];

    if( ch0Count == 0 )
    {
        motorSpeed = 0;
    }
    else
    {
        motorSpeed = (48.0e6/128)/ch0Count * 2 * 3.1415926;
    }
}

void serialTeensy::sendData(char data)
{
    char sendBuffer[2] = {};
    sendBuffer[0] = data;
    serialPort->write(&sendBuffer[0],1);
}
