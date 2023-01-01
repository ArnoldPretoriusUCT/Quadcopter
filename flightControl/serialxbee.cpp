#include "serialxbee.h"

double serialXbee::dT,
       serialXbee::frequency,
       serialXbee::receiveTime,
       serialXbee::sendTime,
       serialXbee::startTime;

double serialXbee::masterServoCommand;

quint8 serialXbee::packetID;

void serialXbee::initialise()
{
    serialPort = new QSerialPort();
    serialPort->setPortName(SERIAL_XBEE_COM_PORT);
//    serialPort->setPort("FT231X USB UART");
    serialPort->open(QIODevice::ReadWrite);
    serialPort->setBaudRate(SERIAL_XBEE_BAUD_RATE,QSerialPort::AllDirections);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);
//    serialPort->setFlowControl(QSerialPort::HardwareControl);
//    serialPort->setReadBufferSize(200);
    serialPort->clear(QSerialPort::AllDirections);

    QObject::connect( serialPort, SIGNAL(readyRead()), this, SLOT(getData()) );

    sendTimer = new QTimer(this);
    connect( sendTimer, SIGNAL(timeout()),this,SLOT(sendData()) );
    sendTimer->setTimerType(Qt::PreciseTimer);
//    sendTimer->start(10);

    qDebug()<<"From xbee thread: "<< QThread::currentThreadId();

    receiveTime_past = 0;
    packetID = 0;
    startTime = QDateTime::currentDateTime().toMSecsSinceEpoch();
}

void serialXbee::sendData()
{
    sendBuffer[0] = '*';
//    sendBuffer[1] = '#';
//    sendBuffer[SERIAL_XBEE_SEND_BUFFER_SIZE-2] = '@';
    sendBuffer[SERIAL_XBEE_SEND_BUFFER_SIZE-1] = '!';

    unsigned char index = 1;
    //ekf quaternion
    double2chars( 0,SERIAL_XBEE_QUAT_SF, &sendBuffer[index] );                index += 2;
    double2chars( 0,SERIAL_XBEE_QUAT_SF, &sendBuffer[index] );                index += 2;
    double2chars( 0,SERIAL_XBEE_QUAT_SF, &sendBuffer[index] );                index += 2;
    double2chars( 0,SERIAL_XBEE_QUAT_SF, &sendBuffer[index] );                index += 2;
    //motor speed ref
    double2chars( motorSpeedControl::reference,1, &sendBuffer[index] );       index += 1;
    //state vector
    char stateVector = (stateMachine::resetMicro << 6)              |
                       (motorSpeedControl::dwEnabled << 5)          |
                       (motorSpeedControl::torqueFeedForward << 4)  |
                       (stateMachine::ledsOn << 3)                  |
                       (stateMachine::heaveControl << 2)            |
                       (stateMachine::calibrationMode << 1)         |
                       (motorSpeedControl::isClosedLoop);
    sendBuffer[index] = stateVector;                                          index++;
    //master servo
//    double2chars( serialXbee::masterServoCommand,1, &sendBuffer[index]);                    index += 2;

//    serialPort->clear(QSerialPort::Output);
    serialPort->write(&sendBuffer[0],SERIAL_XBEE_SEND_BUFFER_SIZE);
//    serialPort->write(&sendBuffer[0],1);
    sendTime = QDateTime::currentDateTime().toMSecsSinceEpoch();
    newDataSent = true;
    stateMachine::resetMicro = false;
//    serialPort->waitForBytesWritten(5);
}

void serialXbee::getData()
{
    packetSize = serialPort->bytesAvailable();
//    qDebug() << "packetSize: " << (double)packetSize;

    if( (packetSize <= 0) )
    {
        serialPort->clear(QSerialPort::AllDirections);
//        qDebug() << "bleh " << packetSize;
    }
    else if( (packetSize >= SERIAL_XBEE_RECEIVE_BUFFER_SIZE) )
    {
        receiveBuffer = serialPort->read(SERIAL_XBEE_RECEIVE_BUFFER_SIZE);
//        serialPort->clear(QSerialPort::Input);
        serialPort->clear(QSerialPort::AllDirections);
        newDataReceived = true;
        getOnboardInformation();
        timeStampData();

        sendData();
    }
}

void serialXbee::getOnboardInformation()
{
    if( (receiveBuffer[0] == '*') && (receiveBuffer[SERIAL_XBEE_RECEIVE_BUFFER_SIZE-1] == '!') )
    {
        unsigned char index = 1;
        imu::gyro[0]   = chars2double(receiveBuffer,index,SERIAL_XBEE_GYRO_SF);                             index += 2;
        imu::gyro[1]   = chars2double(receiveBuffer,index,SERIAL_XBEE_GYRO_SF);                             index += 2;
        imu::gyro[2]   = chars2double(receiveBuffer,index,SERIAL_XBEE_GYRO_SF);                             index += 2;
//        imu::accel[0]  = chars2double(receiveBuffer,index,SERIAL_XBEE_ACCEL_SF);                            index += 2;
//        imu::accel[1]  = chars2double(receiveBuffer,index,SERIAL_XBEE_ACCEL_SF);                            index += 2;
//        imu::accel[2]  = chars2double(receiveBuffer,index,SERIAL_XBEE_ACCEL_SF);                            index += 2;
        motorSpeedControl::escCom[0] = uchar2double(receiveBuffer,index,SERIAL_XBEE_MOTOR_ESC_SF);          index++;
        motorSpeedControl::escCom[1] = uchar2double(receiveBuffer,index,SERIAL_XBEE_MOTOR_ESC_SF);          index++;
        attitudeControl::servoDelta[0] = char2double(receiveBuffer,index,SERIAL_XBEE_SERVO_SF);              index++;
        attitudeControl::servoDelta[1] = char2double(receiveBuffer,index,SERIAL_XBEE_SERVO_SF);              index++;
        attitudeControl::servoDelta[2] = char2double(receiveBuffer,index,SERIAL_XBEE_SERVO_SF);              index++;
        attitudeControl::servoDelta[3] = char2double(receiveBuffer,index,SERIAL_XBEE_SERVO_SF);              index++;
        motorSpeedControl::motorSpeed[0] = uchar2double(receiveBuffer,index,SERIAL_XBEE_MOTOR_SPEED_SF);     index++;
        motorSpeedControl::motorSpeed[1] = uchar2double(receiveBuffer,index,SERIAL_XBEE_MOTOR_SPEED_SF);     index++;
        attitudeControl::loopFrequency = (double)( (quint8)receiveBuffer[index] );                          index++;
//        attitudeControl::isClosedLoop = receiveBuffer[index] & 0x01;
        motorSpeedControl::motorsArmed = (receiveBuffer[index] >> 1) & 0x01;                                index++;

        imu::newData = 1;
    }
}

void serialXbee::timeStampData()
{
    receiveTime = QDateTime::currentDateTime().toMSecsSinceEpoch();
//    receiveTime = (double)timer.nsecsElapsed()/1e6;
    dT = receiveTime - receiveTime_past;
    qDebug() << packetSize << dT;
    receiveTime_past = receiveTime;
    if( dT != 0 )
    {
        frequency = 1.0e3/dT;
    }
    else
    {
        frequency = 0.0;
    }
}
