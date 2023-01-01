#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <QFile>
#include <QDateTime>
#include <math.h>
#include <QTimer>
#include <QElapsedTimer>
#include <QFile>
#include <QThread>
#include <QtCore>
#include <QtCore/QDebug>
#include <QtSerialPort/QSerialPortInfo>
#include <QtSerialPort/QSerialPort>
#include <QTextStream>
#include <armadillo>
#include <iostream>

#include <serialPixy.h>
#include <cameraEKF.h>
#include <imu.h>
#include <motorspeedcontrol.h>
#include <attitudecontrol.h>
#include "serialwixel.h"
#include "calibrationmode.h"
#include "ekfsingleblob.h"
//#include "mainwindow.h"

class dataLogger
{
    public:
        static void initialise();
        static void toggle(bool checked);
        static void logData();

        static char logChar[75];
        static bool isLogging;

    private:
        static QFile file;
};

#endif // DATALOGGER_H
