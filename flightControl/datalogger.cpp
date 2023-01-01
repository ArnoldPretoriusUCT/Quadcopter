#include "datalogger.h"

bool dataLogger::isLogging;
char dataLogger::logChar[75];
QFile dataLogger::file;

void dataLogger::initialise()
{
    sprintf (&logChar[0],"/home/arnold/Academia/uMocap_v2/Logs/motionCaptureLog_%d.txt",QDateTime::currentDateTime().toTime_t());
    QString fileName = QString(logChar);
    file.setFileName(fileName);
}

void dataLogger::toggle(bool checked)
{
    isLogging = checked;

    if( isLogging )
    {
        file.open(QIODevice::WriteOnly | QFile::Text);
    }
    else
    {
        QFile::remove("/home/arnold/Academia/uMocap_v2/Logs/motionCaptureLog.txt");
        file.close();
        QString logChar_copy = QString::fromLatin1(logChar);
        QFile::copy(logChar_copy,"/home/arnold/Academia/uMocap_v2/Logs/motionCaptureLog.txt");

        initialise();
    }
}
//144 elements
void dataLogger::logData()
{
    char DATA[1000];
    sprintf(&DATA[0], "%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d \n",
            (int)(motionCapture::receiveTime(0)-motionCapture::startTime),
            (int)motionCapture::sig(0),
            (int)motionCapture::fDet(0),
            (int)motionCapture::Bp(0,0,0), (int)motionCapture::Bp(1,0,0),
            (int)motionCapture::Bp(0,1,0), (int)motionCapture::Bp(1,1,0),
            (int)motionCapture::Bp(0,2,0), (int)motionCapture::Bp(1,2,0),
            (int)motionCapture::Bp(0,3,0), (int)motionCapture::Bp(1,3,0),
            (int)motionCapture::fSizes(0,0,0), (int)motionCapture::fSizes(1,0,0),
            (int)motionCapture::fSizes(0,1,0), (int)motionCapture::fSizes(1,1,0),
            (int)motionCapture::fSizes(0,2,0), (int)motionCapture::fSizes(1,2,0),
            (int)motionCapture::fSizes(0,3,0), (int)motionCapture::fSizes(1,3,0),

            (int)(motionCapture::receiveTime(1)-motionCapture::startTime),
            (int)motionCapture::sig(1),
            (int)motionCapture::fDet(1),
            (int)motionCapture::Bp(0,0,1), (int)motionCapture::Bp(1,0,1),
            (int)motionCapture::Bp(0,1,1), (int)motionCapture::Bp(1,1,1),
            (int)motionCapture::Bp(0,2,1), (int)motionCapture::Bp(1,2,1),
            (int)motionCapture::Bp(0,3,1), (int)motionCapture::Bp(1,3,1),
            (int)motionCapture::fSizes(0,0,1), (int)motionCapture::fSizes(1,0,1),
            (int)motionCapture::fSizes(0,1,1), (int)motionCapture::fSizes(1,1,1),
            (int)motionCapture::fSizes(0,2,1), (int)motionCapture::fSizes(1,2,1),
            (int)motionCapture::fSizes(0,3,1), (int)motionCapture::fSizes(1,3,1),

            (int)(motionCapture::receiveTime(2)-motionCapture::startTime),
            (int)motionCapture::sig(2),
            (int)motionCapture::fDet(2),
            (int)motionCapture::Bp(0,0,2), (int)motionCapture::Bp(1,0,2),
            (int)motionCapture::Bp(0,1,2), (int)motionCapture::Bp(1,1,2),
            (int)motionCapture::Bp(0,2,2), (int)motionCapture::Bp(1,2,2),
            (int)motionCapture::Bp(0,3,2), (int)motionCapture::Bp(1,3,2),
            (int)motionCapture::fSizes(0,0,2), (int)motionCapture::fSizes(1,0,2),
            (int)motionCapture::fSizes(0,1,2), (int)motionCapture::fSizes(1,1,2),
            (int)motionCapture::fSizes(0,2,2), (int)motionCapture::fSizes(1,2,2),
            (int)motionCapture::fSizes(0,3,2), (int)motionCapture::fSizes(1,3,2),

            (int)(motionCapture::receiveTime(3)-motionCapture::startTime),
            (int)motionCapture::sig(3),
            (int)motionCapture::fDet(3),
            (int)motionCapture::Bp(0,0,3), (int)motionCapture::Bp(1,0,3),
            (int)motionCapture::Bp(0,1,3), (int)motionCapture::Bp(1,1,3),
            (int)motionCapture::Bp(0,2,3), (int)motionCapture::Bp(1,2,3),
            (int)motionCapture::Bp(0,3,3), (int)motionCapture::Bp(1,3,3),
            (int)motionCapture::fSizes(0,0,3), (int)motionCapture::fSizes(1,0,3),
            (int)motionCapture::fSizes(0,1,3), (int)motionCapture::fSizes(1,1,3),
            (int)motionCapture::fSizes(0,2,3), (int)motionCapture::fSizes(1,2,3),
            (int)motionCapture::fSizes(0,3,3), (int)motionCapture::fSizes(1,3,3),

            (int)(motionCapture::receiveTime(4)-motionCapture::startTime),
            (int)motionCapture::sig(4),
            (int)motionCapture::fDet(4),
            (int)motionCapture::Bp(0,0,4), (int)motionCapture::Bp(1,0,4),
            (int)motionCapture::Bp(0,1,4), (int)motionCapture::Bp(1,1,4),
            (int)motionCapture::Bp(0,2,4), (int)motionCapture::Bp(1,2,4),
            (int)motionCapture::Bp(0,3,4), (int)motionCapture::Bp(1,3,4),
            (int)motionCapture::fSizes(0,0,4), (int)motionCapture::fSizes(1,0,4),
            (int)motionCapture::fSizes(0,1,4), (int)motionCapture::fSizes(1,1,4),
            (int)motionCapture::fSizes(0,2,4), (int)motionCapture::fSizes(1,2,4),
            (int)motionCapture::fSizes(0,3,4), (int)motionCapture::fSizes(1,3,4),

            (int)(cameraEKF::x(0)*1e5),(int)(cameraEKF::x(1)*1e5),(int)(cameraEKF::x(2)*1e5),
            (int)(cameraEKF::x(6)*1e5),(int)(cameraEKF::x(7)*1e5),(int)(cameraEKF::x(8)*1e5),(int)(cameraEKF::x(9)*1e5),

            (int)(imu::gyro[0]*1e5),(int)(imu::gyro[1]*1e5),(int)(imu::gyro[2]*1e5),
            (int)(attitudeControl::angleReference(0)*1e5),(int)(attitudeControl::angleReference(1)*1e5),(int)(attitudeControl::angleReference(2)*1e5),
            (int)(motorSpeedControl::reference),(int)(motorSpeedControl::current[0]*1e5),(int)(motorSpeedControl::current[1]*1e5),(int)(motorSpeedControl::motorSpeed[0]*1e5),(int)(motorSpeedControl::motorSpeed[1]*1e5),
            (int)(attitudeControl::servoDelta[0]),(int)(attitudeControl::servoDelta[1]),(int)(attitudeControl::servoDelta[2]),(int)(attitudeControl::servoDelta[3]),(int)(positionControl::heaveCommand*1e5),
            (int)(attitudeControl::quaternionReference(0)*1e5),(int)(attitudeControl::quaternionReference(1)*1e5),(int)(attitudeControl::quaternionReference(2)*1e5),(int)(attitudeControl::quaternionReference(3)*1e5),
            (int)(attitudeControl::quaternion(0)*1e5),(int)(attitudeControl::quaternion(1)*1e5),(int)(attitudeControl::quaternion(2)*1e5),(int)(attitudeControl::quaternion(3)*1e5),(int)(ekfSingleBlob::x(4)*1e5),(int)(ekfSingleBlob::x(5)*1e5),(int)(0),
            (int)(serialWixel::dT*1e5),(int)(serialWixel::receiveTime-motionCapture::startTime),(int)((cameraEKF::iterationTime-motionCapture::startTime)),
            (int)motionCapture::newData[0],(int)motionCapture::newData[1],(int)motionCapture::newData[2],(int)motionCapture::newData[3],(int)motionCapture::newData[4],(int)imu::newData,
            (int)(motorSpeedControl::current[2]*1e5),(int)(motorSpeedControl::current[3]*1e5),(int)(motorSpeedControl::motorSpeed[2]*1e5),(int)(motorSpeedControl::motorSpeed[3]*1e5),(int)(stateMachine::thrust*1e5),(int)(motorSpeedControl::batteryVoltage*1e5)
            );

    QTextStream out(&file);
    out << QString(DATA);
    file.flush();
}

