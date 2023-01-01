#include "serialPixy.h"

bool serialPixy::commsIssue;

void serialPixy::initialise(QString comPort, qint32 baudRate)
{
    serialPort = new QSerialPort();
    serialPort->setPortName(comPort);
    serialPort->open(QIODevice::ReadOnly);
    serialPort->setBaudRate(baudRate,QSerialPort::AllDirections);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    QObject::connect( serialPort, SIGNAL(readyRead()), this, SLOT(getData()) );
    serialPort->clear(QSerialPort::AllDirections);

    watchDogTimer = new QTimer(this);
    connect( watchDogTimer, SIGNAL(timeout()),this,SLOT(watchDog()) );
    watchDogTimer->setTimerType(Qt::PreciseTimer);
    watchDogTimer->start(20);

    dT = 0;
}

void serialPixy::reset(QString comPort, qint32 baudRate)
{
    serialPort->close();
    serialPort = new QSerialPort();
    serialPort->setPortName(comPort);
    serialPort->open(QIODevice::ReadOnly);
    serialPort->setBaudRate(baudRate,QSerialPort::AllDirections);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    QObject::connect( serialPort, SIGNAL(readyRead()), this, SLOT(getData()) );
    serialPort->clear(QSerialPort::AllDirections);
}

void serialPixy::sendData()
{
    serialPort->write(&sendBuffer[0],1);
    if( (int)sendBuffer[0] == 1)
    {
        sendTime = (double)elapsedTimer.nsecsElapsed()/1e6;
    }
}

void serialPixy::getData()
{
    packetSize = serialPort->bytesAvailable();
    if( (int)packetSize <= 0 )
    {
        serialPort->clear(QSerialPort::Input);
    }
    else
    {
        receiveBuffer = serialPort->readAll();
//        serialPort->clear(QSerialPort::Input);
        watchDogCount = 0;
        newData = true;
        timeStampData();
        getObjectInformation();
    }
}

void serialPixy::getObjectInformation()
{
    objectsDetected = 0;
    unsigned char index = 6;
    while(index < 100)
    {
        if( packetSize <= 2 ) break;

        //OBJECT 1
        blob[0].checkSum          = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[0].signature         = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[0].pixelPosition[0]  = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[0].pixelPosition[1]  = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[0].pixelWidth        = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[0].pixelHeight       = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=4;

        if( blob[0].signature != 0  ) objectsDetected++;
        if(index >= packetSize) break;
        //OBJECT 2
        blob[1].checkSum          = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[1].signature         = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[1].pixelPosition[0]  = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[1].pixelPosition[1]  = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[1].pixelWidth        = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[1].pixelHeight       = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=4;

        if( blob[1].signature != 0  ) objectsDetected++;
        if(index >= packetSize) break;
        //OBJECT 3
        blob[2].checkSum          = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[2].signature         = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[2].pixelPosition[0]  = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[2].pixelPosition[1]  = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[2].pixelWidth        = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[2].pixelHeight       = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=4;

        if( blob[2].signature != 0  ) objectsDetected++;
        if(index >= packetSize) break;
        //OBJECT 4
        blob[3].checkSum          = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[3].signature         = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[3].pixelPosition[0]  = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[3].pixelPosition[1]  = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[3].pixelWidth        = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=2;
        blob[3].pixelHeight       = (quint8)receiveBuffer[index] | ( (quint16)receiveBuffer[index+1] << 8 ); index+=4;

        if( blob[3].signature != 0  ) objectsDetected++;
        if(index >= packetSize) break;
        if(objectsDetected >= 4) break;
    }
}

void serialPixy::timeStampData()
{
    receiveTime = (double)elapsedTimer.nsecsElapsed()*1e-6;
    dT = receiveTime - receiveTime_past;

    receiveTime_past = receiveTime;
    if( (abs(dT-20) >= 5))
    {
//        qDebug() << "Warning: serialPixy dt: " << dT;
        commsIssue = true;
    }
}

void serialPixy::watchDog()
{
    watchDogCount++;
    if( watchDogCount >= SERIAL_PIXY_WATCHDOG_THRESHOLD )
    {
//        qDebug() << "Critical issue: serialPixy watchDog timeout!";
        watchDogCount -= watchDogCount;
        dT = 0;
        commsIssue = true;
    }
}
