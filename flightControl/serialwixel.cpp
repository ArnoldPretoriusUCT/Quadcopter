#include "serialwixel.h"

double serialWixel::dT,
              serialWixel::dTMean,
              serialWixel::dTCount,
              serialWixel::frequency,
             serialWixel::receiveTime,
             serialWixel::sendTime,
             serialWixel::startTime;

double serialWixel::masterServoCommand;

quint8 serialWixel::packetID;

bool serialWixel::commsIssue;
unsigned char serialWixel::watchDogCount;

void serialWixel::initialise()
{
     QThread::msleep(100);

    serialPortRx = new QSerialPort();
    serialPortRx->setPortName(SERIAL_WIXEL_COM_PORT_RX);
    serialPortRx->open(QIODevice::ReadOnly);
    serialPortRx->setBaudRate(SERIAL_WIXEL_BAUD_RATE,QSerialPort::Input);
    serialPortRx->setParity(QSerialPort::NoParity);
    serialPortRx->setStopBits(QSerialPort::OneStop);
    serialPortRx->setDataBits(QSerialPort::Data8);
    serialPortRx->setFlowControl(QSerialPort::NoFlowControl);
    serialPortRx->clear(QSerialPort::AllDirections);
    QObject::connect( serialPortRx, SIGNAL(readyRead()), this, SLOT(getData()) );

    serialPortTx = new QSerialPort();
    serialPortTx->setPortName(SERIAL_WIXEL_COM_PORT_TX);\
    serialPortTx->open(QIODevice::WriteOnly);
    serialPortTx->setBaudRate(SERIAL_WIXEL_BAUD_RATE,QSerialPort::Output);
    serialPortTx->setParity(QSerialPort::NoParity);
    serialPortTx->setStopBits(QSerialPort::OneStop);
    serialPortTx->setDataBits(QSerialPort::Data8);
    serialPortTx->setFlowControl(QSerialPort::NoFlowControl);
    serialPortTx->clear(QSerialPort::AllDirections);

    sendTimer = new QTimer(this);
    connect( sendTimer, SIGNAL(timeout()),this,SLOT(sendData()) );
    sendTimer->setTimerType(Qt::PreciseTimer);
//    sendTimer->start(10);

    receiveTime_past = sendTime_past = 0;
    packetID = 0;
    dTMean = 0;
    dTCount = 1;
    commsIssue = false;
}

void serialWixel::sendData()
{
    sendBuffer[0] = '*';
    sendBuffer[SERIAL_WIXEL_SEND_BUFFER_SIZE-1] = '!';

    unsigned char index = 1;
    //heave command
    double2chars( positionControl::heaveCommand,SERIAL_WIXEL_HEAVE_SF, &sendBuffer[index] );                                    index += 2;
    //reference quaternion vector
     double2chars( attitudeControl::quaternionReference(1),SERIAL_WIXEL_QUATERNION_SF, &sendBuffer[index] );        index += 2;
     double2chars( attitudeControl::quaternionReference(2),SERIAL_WIXEL_QUATERNION_SF, &sendBuffer[index] );        index += 2;
     double2chars( attitudeControl::quaternionReference(3),SERIAL_WIXEL_QUATERNION_SF, &sendBuffer[index] );        index += 2;
   //camera quaternion vector
    double2chars( cameraEKF::x(7),SERIAL_WIXEL_QUATERNION_SF, &sendBuffer[index] );                                                        index += 2;
    double2chars( cameraEKF::x(8),SERIAL_WIXEL_QUATERNION_SF, &sendBuffer[index] );                                                        index += 2;
    double2chars( cameraEKF::x(9),SERIAL_WIXEL_QUATERNION_SF, &sendBuffer[index] );                                                        index += 2;
    //motor speed ref
    sendBuffer[index] = (char)( motorSpeedControl::reference*255/800 );                                                                                         index++;
    //state vector
    char stateVector = ((serialRadio::isOpen && !motionCapture::isCritical) << 7)   |
                                         (attitudeControl::isClosedLoop << 6)                                            |
                                         (stateMachine::ledsOn << 5)                                                           |
                                         (stateMachine::resetMicro << 4)                                                    |
                                         (calibrationMode::isCalibrating << 3)                                           |
                                         (motorSpeedControl::motorsArmed << 2)                                  |
                                         (motorSpeedControl::torqueFeedForward << 1)                      |
                                         (motorSpeedControl::isClosedLoop);
    sendBuffer[index] = stateVector;                                                                                                                                                                index++;


    serialPortTx->clear(QSerialPort::Output);
    serialPortTx->write(&sendBuffer[0],SERIAL_WIXEL_SEND_BUFFER_SIZE);
    serialPortTx->waitForBytesWritten(-1);

    sendTime_past = sendTime;
    sendTime = (double)elapsedTimer.nsecsElapsed()/1e6;
    if( abs(sendTime-sendTime_past-5) >= 3)
    {
//        qDebug() << "Warning: serialWixel tx dt: " << sendTime-sendTime_past;
    }
    newDataSent = true;
    stateMachine::resetMicro = false;

    watchDog();
}

void serialWixel::getData()
{
    packetSize = serialPortRx->bytesAvailable();
    if( packetSize == SERIAL_WIXEL_RECEIVE_BUFFER_SIZE )
    {
        receiveBuffer = serialPortRx->read(SERIAL_WIXEL_RECEIVE_BUFFER_SIZE);
        serialPortRx->clear(QSerialPort::AllDirections);
//        receiveBuffer = serialPort->read(SERIAL_WIXEL_RECEIVE_BUFFER_SIZE);
//        serialPort->clear(QSerialPort::Input);
        newDataReceived = true;
        getOnboardInformation();
        sendData();
    }
    else if( packetSize > SERIAL_WIXEL_RECEIVE_BUFFER_SIZE )
    {
        serialPortRx->clear(QSerialPort::AllDirections);
//        serialPort->clear(QSerialPort::Input);
    }
}

void serialWixel::getOnboardInformation()
{
    if( (receiveBuffer[0] == '*') && (receiveBuffer[SERIAL_WIXEL_RECEIVE_BUFFER_SIZE-1] == '!') )
    {
        watchDogCount = 0;
        commsIssue = false;

        unsigned char index = 1;
//        imu::gyro[0] = chars2double(receiveBuffer,index,SERIAL_WIXEL_GYRO_SF);                                                                  index += 2;
        imu::gyro[0] = uchars2double(receiveBuffer,index,1)/1e5;                                                                                                    index += 2;
        imu::gyro[1] = chars2double(receiveBuffer,index,SERIAL_WIXEL_GYRO_SF);                                                                  index += 2;
//        motorSpeedControl::motorSpeed[2] = chars2double(receiveBuffer,index,40);                                                                  index += 2;
        imu::gyro[2] = chars2double(receiveBuffer,index,SERIAL_WIXEL_GYRO_SF);                                                                  index += 2;
//        motorSpeedControl::escCom[2] = chars2double(receiveBuffer,index,1);                                                                          index += 2;
        attitudeControl::quaternion(1) = chars2double(receiveBuffer,index,SERIAL_WIXEL_QUATERNION_SF);                 index += 2;
        attitudeControl::quaternion(2) = chars2double(receiveBuffer,index,SERIAL_WIXEL_QUATERNION_SF);                 index += 2;
        attitudeControl::quaternion(3) = chars2double(receiveBuffer,index,SERIAL_WIXEL_QUATERNION_SF);                 index += 2;
        attitudeControl::quaternion(0) = cos(asin(norm(attitudeControl::quaternion.rows(1,3))));
        motorSpeedControl::current[0] = uchar2double(receiveBuffer,index,SERIAL_WIXEL_MOTOR_CURRENT_SF);      index++;
        motorSpeedControl::current[1] = uchar2double(receiveBuffer,index,SERIAL_WIXEL_MOTOR_CURRENT_SF);      index++;
        motorSpeedControl::current[2] = uchar2double(receiveBuffer,index,SERIAL_WIXEL_MOTOR_CURRENT_SF);      index++;
        motorSpeedControl::current[3] = uchar2double(receiveBuffer,index,SERIAL_WIXEL_MOTOR_CURRENT_SF);      index++;
        attitudeControl::servoDelta[0] = char2double(receiveBuffer,index,SERIAL_WIXEL_SERVO_SF);                                index++;
        attitudeControl::servoDelta[1] = char2double(receiveBuffer,index,SERIAL_WIXEL_SERVO_SF);                                index++;
        attitudeControl::servoDelta[2] = char2double(receiveBuffer,index,SERIAL_WIXEL_SERVO_SF);                                index++;
        attitudeControl::servoDelta[3] = char2double(receiveBuffer,index,SERIAL_WIXEL_SERVO_SF);                                index++;
        motorSpeedControl::motorSpeed[0] = uchar2double(receiveBuffer,index,SERIAL_WIXEL_MOTOR_SPEED_SF);   index++;
        motorSpeedControl::motorSpeed[1] = uchar2double(receiveBuffer,index,SERIAL_WIXEL_MOTOR_SPEED_SF);   index++;
        motorSpeedControl::motorSpeed[2] = uchar2double(receiveBuffer,index,SERIAL_WIXEL_MOTOR_SPEED_SF);   index++;
        motorSpeedControl::motorSpeed[3] = uchar2double(receiveBuffer,index,SERIAL_WIXEL_MOTOR_SPEED_SF);   index++;
        motorSpeedControl::batteryVoltage = uchar2double(receiveBuffer,index,SERIAL_WIXEL_V_BAT_SF);

        imu::newData = true;
        timeStampData();
        motorSpeedControl::checkSaturation();
        for( int i=0;i<4;i++ )
        {
            if( abs(attitudeControl::servoDelta[i]) == attitudeControl::servoDeltaMax )
            {
//                qDebug() << "Warning: servo saturation warning";
            }
        }
    }
    receiveBuffer_past = receiveBuffer;
}

void serialWixel::timeStampData()
{
    receiveTime = (double)elapsedTimer.nsecsElapsed()*1e-6;
    dT = receiveTime - receiveTime_past;

    dTMean = ((dTCount-1)*dTMean+dT)/dTCount;
    dTCount++;

    receiveTime_past = receiveTime;
    if( dT != 0 )
    {
        frequency = 1.0e3/dT;
    }
    else
    {
        frequency = 0.0;
    }
    if( abs(dT-SERIAL_WIXEL_DT_MSEC  ) >= 2  )
    {
        commsIssue = true;
        qDebug() << "WARNING: SERIAL RX DT: " << dT;
    }
}

void serialWixel::watchDog()
{
    if( watchDogCount >= 100 )
    {
//        commsIssue = true;
        frequency = dT = 0.0;
        watchDogCount = 100;
    }
    watchDogCount++;
}
