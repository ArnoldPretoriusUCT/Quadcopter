 #ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <math.h>
#include <QTimer>
#include <QElapsedTimer>
#include <QFile>
#include <QThread>
#include <QtCore>
#include <QtCore/QDebug>
#include <QtSerialPort/QSerialPortInfo>
#include <QtSerialPort/QSerialPort>
#include <armadillo>
#include <iostream>

#include <unistd.h>
#include <sys/resource.h>

#include "serialPixy.h"
//#include "serialxbee.h"
#include "serialwixel.h"
#include "serialradio.h"
#include "motioncapture.h"
#include "cameraEKF.h"
#include "qcustomplot.h"
#include "common.h"
#include "statemachine.h"
#include "attitudecontrol.h"
#include "motorspeedcontrol.h"
#include "imu.h"
#include "positioncontrol.h"
#include "calibrationmode.h"

#define MAINWINDOW_GUI_DT    1000 //msec
#define MAINWINDOW_PLOT_DT  50 //msec

using namespace arma;
using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    static double motorSpeedRef;
    static bool heaveControl;
    static bool motorSpeedControl;
    static bool calibrationMode;
    static double shutterDelay[4];

    QThread *wixelThread,*radioThread,*mocapThread;
    serialWixel *wixel;
    serialRadio *radio;
    motionCapture *mocap;

public slots:
    void updateGUI();
//    void updateData();
    void initialiseThreads();

    void setupGraphs(QCustomPlot *customPlot1,QCustomPlot *customPlot2,QCustomPlot *customPlot3,QCustomPlot *customPlot4,QCustomPlot *customPlot5,QCustomPlot *customPlot6,
                     QCustomPlot *customPlot7,QCustomPlot *customPlot8,QCustomPlot *customPlot9,QCustomPlot *customPlot10,QCustomPlot *customPlot11,QCustomPlot *customPlot12,
                     QCustomPlot *customPlot13);

    void plotGraphs();

private slots:
    void on_checkBox_clicked(bool checked);

    void on_pushButton_clicked();

    void on_checkBox_6_clicked(bool checked);

    void on_checkBox_2_clicked(bool checked);

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_checkBox_10_clicked();

    void on_checkBox_14_clicked(bool checked);

    void on_pushButton_6_clicked();

private:
    Ui::MainWindow *ui;
    FILE *f1;

    QPalette pal_warning,pal_original;
};


#endif // MAINWINDOW_H
