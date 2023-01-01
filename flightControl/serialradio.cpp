#include "serialradio.h"

double serialRadio::receiveTime,
       serialRadio::receiveTime_past,
       serialRadio::dT,
       serialRadio::dTMean,
       serialRadio::dTCount,
       serialRadio::frequency,
       serialRadio::controllerCommand[3];

bool serialRadio::isOpen,
     serialRadio::commsIssue;
unsigned char serialRadio::watchDogCount;

void serialRadio::initialise()
{
    serialPort = new QSerialPort();
    serialPort->setPortName(RADIO_PORT_NAME);
    serialPort->open(QIODevice::ReadOnly);
    serialPort->setBaudRate(RADIO_BAUD_RATE,QSerialPort::Input);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);
    serialPort->clear(QSerialPort::AllDirections);

    QObject::connect( serialPort, SIGNAL(readyRead()), this, SLOT(getData()) );

    packetSize = 0;

    watchDogTimer = new QTimer(this);
    connect( watchDogTimer, SIGNAL(timeout()),this,SLOT(watchDog()) );
    watchDogTimer->setTimerType(Qt::PreciseTimer);
    watchDogTimer->start(20);
}

void serialRadio::getData()
{
    packetSize = serialPort->bytesAvailable();
//    qDebug() << packetSize;
    if( packetSize >= RADIO_BUFFER_SIZE )
    {
        receiveBuffer = serialPort->read(RADIO_BUFFER_SIZE);
        serialPort->clear(QSerialPort::Input);
        timeStampData();
        newData = true;
        serialRadio::watchDogCount = 0;

        quint16 spektrum_word;
        if ( (receiveBuffer[1] == 0xA2) && (receiveBuffer[0] == 0x03) )
        {
            if ( (receiveBuffer[2] & 0xFC) == RADIO_SPEKTRUM_CH1 )
            {
                spektrum_word = ((quint16)receiveBuffer[2] << 8) & 0x03FF;
                channelValue[1] = (quint16)(spektrum_word | (quint8)receiveBuffer[3]);
            }
            if ( (receiveBuffer[4] & 0xFC) == RADIO_SPEKTRUM_CH5 )
            {
                spektrum_word = ((quint16)receiveBuffer[4] << 8) & 0x03FF;
                channelValue[5] = (quint16)(spektrum_word | (quint8)receiveBuffer[5]);
            }
            if ( (receiveBuffer[6] & 0xFC) == RADIO_SPEKTRUM_CH2 )
            {
                spektrum_word = ((quint16)receiveBuffer[6] << 8) & 0x03FF;
                channelValue[2] = (quint16)(spektrum_word | (quint8)receiveBuffer[7]);
            }
            if ( (receiveBuffer[8] & 0xFC) == RADIO_SPEKTRUM_CH3 )
            {
                spektrum_word = ((quint16)receiveBuffer[8] << 8) & 0x03FF;
                channelValue[3] = (quint16)(spektrum_word | (quint8)receiveBuffer[9]);
            }
            if ( (receiveBuffer[10] & 0xFC) == RADIO_SPEKTRUM_CH0 )
            {
                spektrum_word = ((quint16)receiveBuffer[10] << 8) & 0x03FF;
                channelValue[0] = (quint16)(spektrum_word | (quint8)receiveBuffer[11]);
            }
            if ( (receiveBuffer[14] & 0xFC) == RADIO_SPEKTRUM_CH4 )
            {
                spektrum_word = ((quint16)receiveBuffer[14] << 8) & 0x03FF;
                channelValue[4] = (quint16)(spektrum_word | (quint8)receiveBuffer[15]);
            }

            if( isInitialised == false)
            {
                getNominalChannelValues();
                isInitialised = true;
            }
            getControllerCommands();
//            qDebug() << channelValue[5] << channelValueNom[5];
        }
    }
}

void serialRadio::timeStampData()
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
    if( abs(dT-21) >= 5  )
    {
//        qDebug() << 'serialRadio.dT: irregular dT:' << dT;
        commsIssue = true;
    }
}

void serialRadio::watchDog()
{
    serialRadio::watchDogCount++;
    if( watchDogCount >= RADIO_WATCHDOG_THRESHOLD )
    {
        isOpen = false;
        watchDogCount = RADIO_WATCHDOG_THRESHOLD;

//        qDebug() << "serialRadio.getData: no comms";
        frequency = 0.0;
        dT = 0.0;
        commsIssue = true;
    }
    else
    {
        serialRadio::isOpen = true;
    }
}

void serialRadio::getControllerCommands()
{
    controllerCommand[0] =    (double)channelValue[4]/RADIO_VALUE_RANGE_CH4; //heave
    controllerCommand[1] = -2*((double)channelValue[1]-channelValueNom[1])/RADIO_VALUE_RANGE_CH1; //roll
    controllerCommand[2] =  2*((double)channelValue[2]-channelValueNom[2])/RADIO_VALUE_RANGE_CH2; //pitch


    positionControl::heaveCommand = POSITIONCONTROL_THRUST_MAX*controllerCommand[0];
    if( positionControl::heaveCommand < POSITIONCONTROL_THRUST_MIN )
    {
        positionControl::heaveCommand = POSITIONCONTROL_THRUST_MIN;
    }

    if( channelValue[3] != channelValueNom[3])
    {
        attitudeControl::isClosedLoop = !attitudeControl::isClosedLoop;
        channelValueNom[3] = channelValue[3];
    }    
    quint8 cntMax = 25;
    static quint8 cnt = cntMax;
    if( (channelValue[5] != channelValueNom[5]) && (cnt==cntMax) )
    {
        motorSpeedControl::motorsArmed = !motorSpeedControl::motorsArmed;
        cnt = 0;
    }
    cnt++;
    if( cnt>=cntMax ) cnt = cntMax;
}

void serialRadio::getNominalChannelValues()
{
    for(int i=0;i<6;i++)
    {
        channelValueNom[i] = channelValue[i];
    }
}
