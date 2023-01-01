#include "mainwindow.h"
#include "ui_mainwindow.h"

double MainWindow::motorSpeedRef;
bool MainWindow::heaveControl;
bool MainWindow::motorSpeedControl;
bool MainWindow::calibrationMode;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("Flight Control");

    foreach ( const QSerialPortInfo &info, QSerialPortInfo::availablePorts() )
    {
        qDebug() << "Name           :" << info.portName();
        qDebug() << "Description    :" << info.description();
        qDebug() << "Manufacturer   :" << info.manufacturer();
    }

    initialiseThreads();

    setupGraphs(ui->customPlot1,ui->customPlot2,ui->customPlot3,ui->customPlot4,ui->customPlot5,ui->customPlot6,
                              ui->customPlot7,ui->customPlot8,ui->customPlot9,ui->customPlot10,ui->customPlot11,ui->customPlot12,
                              ui->customPlot13);

    QTimer *guiTimer = new QTimer(this);
    connect( guiTimer, SIGNAL(timeout()),this,SLOT(updateGUI()) );
    guiTimer->setTimerType(Qt::PreciseTimer);
    guiTimer->start(MAINWINDOW_GUI_DT);
    QTimer *plotTimer = new QTimer(this);
    connect( plotTimer, SIGNAL(timeout()),this,SLOT(plotGraphs()) );
    plotTimer->setTimerType(Qt::PreciseTimer);
    plotTimer->start(MAINWINDOW_PLOT_DT); //msec

    elapsedTimer.start();

    pal_warning.setColor(QPalette::Background, Qt::red);
    pal_original.setColor(QPalette::Background, Qt::white);
    ui->centralWidget->setAutoFillBackground(true);
    ui->horizontalSlider_2->setValue(MOTORSPEEDCONTROL_NOMINAL_SETPOINT);
    ui->horizontalSlider_3->setMaximum(POSITIONCONTROL_THRUST_MAX);
    ui->verticalSlider->setMaximum(ATTITUDECONTROL_SERVODELTA_MAX);
    ui->verticalSlider_2->setMaximum(ATTITUDECONTROL_SERVODELTA_MAX);
    ui->verticalSlider_3->setMaximum(ATTITUDECONTROL_SERVODELTA_MAX);
    ui->verticalSlider_4->setMaximum(ATTITUDECONTROL_SERVODELTA_MAX);
    ui->verticalSlider->setMinimum(-ATTITUDECONTROL_SERVODELTA_MAX);
    ui->verticalSlider_2->setMinimum(-ATTITUDECONTROL_SERVODELTA_MAX);
    ui->verticalSlider_3->setMinimum(-ATTITUDECONTROL_SERVODELTA_MAX);
    ui->verticalSlider_4->setMinimum(-ATTITUDECONTROL_SERVODELTA_MAX);

    cameraEKF::cameraDataOnly = ui->checkBox_14->isChecked();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateGUI()
{
    motionCapture::watchDog();
    //comms issues
    ui->radioButton_3->setChecked(serialWixel::commsIssue);
    ui->radioButton->setChecked(serialRadio::commsIssue);
    ui->radioButton_2->setChecked(serialPixy::commsIssue);
    serialWixel::commsIssue = false;
    serialRadio::commsIssue = false;
    serialPixy::commsIssue = false;

    //camera ekf
    cameraEKF::isActive = ui->checkBox_10->isChecked();
    ekfSingleBlob::isActive = cameraEKF::isActive;
    //ekf type
    motionCapture::ekfType = ui->checkBox_11->isChecked();
    //state vector
    stateMachine::heaveControl = ui->checkBox_5->isChecked();
    motorSpeedControl::isClosedLoop = ui->checkBox_2->isChecked();
    stateMachine::calibrationMode = ui->checkBox_6->isChecked();
    if( stateMachine::calibrationMode == false )
    {
        ui->horizontalSlider->setDisabled(true);
    }
    else
    {
        ui->horizontalSlider->setEnabled(true);
    }
    motorSpeedControl::torqueFeedForward = ui->checkBox_7->isChecked();
    stateMachine::ledsOn = ui->checkBox_8->isChecked();
    //motor
    motorSpeedControl::reference = ui->horizontalSlider_2->value();
    ui->listWidget_21->item(0)->setText(QString::number(motorSpeedControl::motorSpeed[0]));
    ui->listWidget_21->item(1)->setText(QString::number(motorSpeedControl::motorSpeed[1]));
    ui->listWidget_21->item(2)->setText(QString::number(motorSpeedControl::motorSpeed[2]));
    ui->listWidget_21->item(3)->setText(QString::number(motorSpeedControl::motorSpeed[3]));

    //CAM1
    ui->listWidget_6->clear();
    ui->listWidget_6->insertItem(0,QString::number(1));
    ui->listWidget_6->insertItem(1,QString::number(motionCapture::dTMean(0),'f',1));
    ui->listWidget_6->insertItem(2,QString::number(motionCapture::fDetMean(0),'f',1));
    ui->listWidget_6->insertItem(3,QString::number(motionCapture::camValidMean(0),'f',1));
    //CAM2
    ui->listWidget_7->clear();
    ui->listWidget_7->insertItem(0,QString::number(2));
    ui->listWidget_7->insertItem(1,QString::number(motionCapture::dTMean(1),'f',1));
    ui->listWidget_7->insertItem(2,QString::number(motionCapture::fDetMean(1),'f',1));
    ui->listWidget_7->insertItem(3,QString::number(motionCapture::camValidMean(1),'f',1));
    //CAM3
    ui->listWidget_8->clear();
    ui->listWidget_8->insertItem(0,QString::number(3));
    ui->listWidget_8->insertItem(1,QString::number(motionCapture::dTMean(2),'f',1));
    ui->listWidget_8->insertItem(2,QString::number(motionCapture::fDetMean(2),'f',1));
    ui->listWidget_8->insertItem(3,QString::number(motionCapture::camValidMean(2),'f',1));
    //CAM4
    ui->listWidget_9->clear();
    ui->listWidget_9->insertItem(0,QString::number(4));
    ui->listWidget_9->insertItem(1,QString::number(motionCapture::dTMean(3),'f',1));
    ui->listWidget_9->insertItem(2,QString::number(motionCapture::fDetMean(3),'f',1));
    ui->listWidget_9->insertItem(3,QString::number(motionCapture::camValidMean(3),'f',1));
    //CAM5
    ui->listWidget_10->clear();
    ui->listWidget_10->insertItem(0,QString::number(5));
    ui->listWidget_10->insertItem(1,QString::number(motionCapture::dTMean(4),'f',1));
    ui->listWidget_10->insertItem(2,QString::number(motionCapture::fDetMean(4),'f',1));
    ui->listWidget_10->insertItem(3,QString::number(motionCapture::camValidMean(4),'f',1));
    motionCapture::fValid.zeros(5);
    motionCapture::dTMean.zeros(5); motionCapture::fDetMean.zeros(5); motionCapture::camValidMean.zeros(5); motionCapture::camValidCount.ones(5); motionCapture::meanCount.ones(5);

    //Motor speed ref + master servo
    ui->label_4->setText(QString::number(ui->horizontalSlider_2->value()));
    ui->label_5->setText(QString::number(ui->horizontalSlider->value()));
    calibrationMode::servoCommand = ui->horizontalSlider->value();
    ui->horizontalSlider_2->setPageStep(ui->listWidget_22->item(0)->text().toInt());
    ui->horizontalSlider->setPageStep(ui->listWidget_22->item(1)->text().toInt());
    //Heave command
    ui->horizontalSlider_3->setValue(positionControl::heaveCommand);
    ui->label_18->setText(QString::number(positionControl::heaveCommand));

    //Auto-logging
    if(  ui->checkBox_13->isChecked()   )
    {
        if( (positionControl::heaveCommand > POSITIONCONTROL_THRUST_MIN) && !dataLogger::isLogging )
        {
            dataLogger::toggle(true);
            ui->checkBox->setChecked(true);
        }
        else if( (positionControl::heaveCommand == POSITIONCONTROL_THRUST_MIN) && dataLogger::isLogging)
        {
            dataLogger::toggle(false);
            ui->checkBox->setChecked(false);
        }
    }

    //Servos
    if(  ui->checkBox_9->isChecked() )
    {
        ui->verticalSlider->setValue(attitudeControl::servoDelta[0]);
        ui->verticalSlider_2->setValue(attitudeControl::servoDelta[1]);
        ui->verticalSlider_3->setValue(attitudeControl::servoDelta[2]);
        ui->verticalSlider_4->setValue(attitudeControl::servoDelta[3]);
    }
    else if( ui->checkBox_9->isChecked() == false )
    {
        ui->verticalSlider->setEnabled(false);
        ui->verticalSlider_2->setEnabled(false);
        ui->verticalSlider_3->setEnabled(false);
        ui->verticalSlider_4->setEnabled(false);
    }
    ui->label_6->setText(QString::number(ui->verticalSlider->value()));
    ui->label_7->setText(QString::number(ui->verticalSlider_2->value()));
    ui->label_8->setText(QString::number(ui->verticalSlider_3->value()));
    ui->label_9->setText(QString::number(ui->verticalSlider_4->value()));
    //Position data
    ui->listWidget_16->clear();
    ui->listWidget_16->insertItem(0,QString::number(cameraEKF::x(0),'f',2));
    ui->listWidget_16->insertItem(1,QString::number(cameraEKF::x(1),'f',2));
    ui->listWidget_16->insertItem(2,QString::number(cameraEKF::x(2),'f',2));
    //Attitude data
    double alpha = 2*acos(cameraEKF::x(6));
    vec angle;
    if( alpha ==0 )
    {
        angle << 0 << 0 << 0 << endr;
    }
    else
    {
        angle =  cameraEKF::x.rows(7,9)*( alpha/sin(alpha/2))*180/M_PI;
    }

    ui->listWidget_27->clear();
    ui->listWidget_27->insertItem(0,QString::number(angle(0),'f',1));
    ui->listWidget_27->insertItem(1,QString::number(angle(1),'f',1));
    ui->listWidget_27->insertItem(2,QString::number(angle(2),'f',1));
    //Frequencies
    ui->listWidget_18->clear();
    ui->listWidget_18->insertItem(0,QString::number(serialWixel::dTMean,'f',1));
    ui->listWidget_18->insertItem(1,QString::number(serialRadio::dTMean,'f',1));
    ui->listWidget_18->insertItem(2,QString::number(cameraEKF::dT*1e3,'f',1));
    ui->listWidget_18->insertItem(3,QString::number(cameraEKF::dT*1e3,'f',1));
    serialWixel::dTMean =  serialRadio::dTMean =0;    serialWixel::dTCount = serialRadio::dTCount = 1;

    //State vector
    ui->radioButton_4->setChecked(motorSpeedControl::motorsArmed);
    ui->radioButton_5->setChecked(attitudeControl::isClosedLoop);
//    //Per-blob valid count
    ui->listWidget_15->clear();
    ui->listWidget_15->insertItem(0,QString::number(5*motionCapture::xyzValidMean(0),'f',1));
    ui->listWidget_15->insertItem(1,QString::number(5*motionCapture::xyzValidMean(1),'f',1));
    ui->listWidget_15->insertItem(2,QString::number(5*motionCapture::xyzValidMean(2),'f',1));
   motionCapture::valid.zeros(3); motionCapture::xyzValidCount = 0;

   //Thrust test data
   ui->label_19->setText(QString::number((stateMachine::thrust)));
   ui->label_23->setText(QString::number((attitudeControl::thrustEstimate(0))));
   QString temp = ui->listWidget_23->item(0)->text();
   attitudeControl::thrustScaleFactor = temp.toDouble();   
   ui->label_27->setText(QString::number((positionControl::heaveCommand-3)*250.0/17+50));

   ui->label_26->setText(QString::number((motorSpeedControl::escCom[2])));
//   qDebug() << attitudeControl::thrustScaleFactor;
   //Battery voltage
   ui->label_22->setText(QString::number(motorSpeedControl::batteryVoltage,'f',1));
}

void MainWindow::initialiseThreads()
{
    QThread::currentThread()->setPriority(QThread::LowPriority);
    //Radio thread
    radioThread = new QThread;
    radio       = new serialRadio;
    radio->moveToThread(radioThread);
    connect(radioThread, SIGNAL(started()), radio, SLOT(initialise()));
    radioThread->start();
    radioThread->setPriority(QThread::NormalPriority);
    //Mocap&EKF%Control thread
    mocapThread = new QThread;
    mocap       = new motionCapture;
    mocap->moveToThread(mocapThread);
    connect(mocapThread, SIGNAL(started()), mocap, SLOT(initialise()));
    mocapThread->start();
    mocapThread->setPriority(QThread::HighestPriority);
    //Wixel thread
    wixelThread = new QThread;
    wixel       = new serialWixel;
    wixel->moveToThread(wixelThread);
    connect(wixelThread, SIGNAL(started()), wixel, SLOT(initialise()));
    wixelThread->start();
    wixelThread->setPriority(QThread::HighPriority);
}

void MainWindow::setupGraphs(QCustomPlot *customPlot1,QCustomPlot *customPlot2,QCustomPlot *customPlot3,QCustomPlot *customPlot4,QCustomPlot *customPlot5,QCustomPlot *customPlot6,QCustomPlot *customPlot7,QCustomPlot *customPlot8, QCustomPlot *customPlot9, QCustomPlot *customPlot10, QCustomPlot *customPlot11, QCustomPlot *customPlot12, QCustomPlot *customPlot13)
{
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%m:%s");

    const quint16 PEN_WIDTH = 1;

    //Customplot1
    customPlot1->addGraph();
    customPlot1->graph(0)->setPen(QPen(Qt::red,PEN_WIDTH));
    customPlot1->addGraph();
    customPlot1->graph(1)->setPen(QPen(Qt::blue,PEN_WIDTH));
    customPlot1->addGraph();
    customPlot1->graph(2)->setPen(QPen(Qt::green,PEN_WIDTH));
    customPlot1->xAxis->setTicker(timeTicker);
    customPlot1->yAxis->setRange(-10, 10);
    customPlot1->xAxis->setLabel("time (s)");
    customPlot1->yAxis->setLabel("position (cm)");    
    customPlot1->axisRect()->setupFullAxesBox(true);
    customPlot1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
//    customPlot1->setOpenGl(true);

    //Customplot2
    customPlot2->addGraph();
    customPlot2->graph(0)->setPen(QPen(Qt::red,PEN_WIDTH));
    customPlot2->addGraph();
    customPlot2->graph(1)->setPen(QPen(Qt::blue,PEN_WIDTH));
    customPlot2->addGraph();
    customPlot2->graph(2)->setPen(QPen(Qt::green,PEN_WIDTH));
    customPlot2->xAxis->setTicker(timeTicker);
    customPlot2->yAxis->setRange(-10, 10);
    customPlot2->xAxis->setLabel("time (s)");
    customPlot2->yAxis->setLabel("angle (deg)");
    customPlot2->axisRect()->setupFullAxesBox(true);
    customPlot2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
//    customPlot2->setOpenGl(true);

    //Customplot3
    customPlot3->addGraph();
    customPlot3->graph(0)->setPen(QPen(Qt::red));
    customPlot3->addGraph();
    customPlot3->graph(1)->setPen(QPen(Qt::blue));
    customPlot3->addGraph();
    customPlot3->graph(2)->setPen(QPen(Qt::green));
    customPlot3->yAxis->setRange(0, 200);
    customPlot3->xAxis->setRange(0, 320);
    customPlot3->xAxis->setLabel("x");
    customPlot3->yAxis->setLabel("y");
    customPlot3->axisRect()->setupFullAxesBox(true);
    customPlot3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    customPlot3->graph(0)->setLineStyle(QCPGraph::lsNone);
    customPlot3->graph(1)->setLineStyle(QCPGraph::lsNone);
    customPlot3->graph(2)->setLineStyle(QCPGraph::lsNone);
    customPlot3->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
    customPlot3->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
    customPlot3->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));


    //Customplot4
    customPlot4->addGraph();
    customPlot4->graph(0)->setPen(QPen(Qt::red));
    customPlot4->addGraph();
    customPlot4->graph(1)->setPen(QPen(Qt::blue));
    customPlot4->addGraph();
    customPlot4->graph(2)->setPen(QPen(Qt::green));
    customPlot4->yAxis->setRange(0, 200);
    customPlot4->xAxis->setRange(0, 320);
    customPlot4->xAxis->setLabel("x");
    customPlot4->yAxis->setLabel("y");
    customPlot4->axisRect()->setupFullAxesBox(true);
    customPlot4->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    customPlot4->graph(0)->setLineStyle(QCPGraph::lsNone);
    customPlot4->graph(1)->setLineStyle(QCPGraph::lsNone);
    customPlot4->graph(2)->setLineStyle(QCPGraph::lsNone);
    customPlot4->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
    customPlot4->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
    customPlot4->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));

    //Customplot5
    customPlot5->addGraph();
    customPlot5->graph(0)->setPen(QPen(Qt::red));
    customPlot5->addGraph();
    customPlot5->graph(1)->setPen(QPen(Qt::blue));
    customPlot5->addGraph();
    customPlot5->graph(2)->setPen(QPen(Qt::green));
    customPlot5->yAxis->setRange(0, 200);
    customPlot5->xAxis->setRange(0, 320);
    customPlot5->xAxis->setLabel("x");
    customPlot5->yAxis->setLabel("y");
    customPlot5->axisRect()->setupFullAxesBox(true);
    customPlot5->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    customPlot5->graph(0)->setLineStyle(QCPGraph::lsNone);
    customPlot5->graph(1)->setLineStyle(QCPGraph::lsNone);
    customPlot5->graph(2)->setLineStyle(QCPGraph::lsNone);
    customPlot5->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
    customPlot5->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
    customPlot5->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));

    //Customplot6
    customPlot6->addGraph();
    customPlot6->graph(0)->setPen(QPen(Qt::red));
    customPlot6->addGraph();
    customPlot6->graph(1)->setPen(QPen(Qt::blue));
    customPlot6->addGraph();
    customPlot6->graph(2)->setPen(QPen(Qt::green));
    customPlot6->yAxis->setRange(0, 200);
    customPlot6->xAxis->setRange(0, 320);
    customPlot6->xAxis->setLabel("x");
    customPlot6->yAxis->setLabel("y");
    customPlot6->axisRect()->setupFullAxesBox(true);
    customPlot6->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    customPlot6->graph(0)->setLineStyle(QCPGraph::lsNone);
    customPlot6->graph(1)->setLineStyle(QCPGraph::lsNone);
    customPlot6->graph(2)->setLineStyle(QCPGraph::lsNone);
    customPlot6->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
    customPlot6->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
    customPlot6->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));

    //Customplot7
    customPlot7->addGraph();
    customPlot7->graph(0)->setPen(QPen(Qt::red));
    customPlot7->addGraph();
    customPlot7->graph(1)->setPen(QPen(Qt::blue));
    customPlot7->addGraph();
    customPlot7->graph(2)->setPen(QPen(Qt::green));
    customPlot7->yAxis->setRange(0, 200);
    customPlot7->xAxis->setRange(0, 320);
    customPlot7->xAxis->setLabel("x");
    customPlot7->yAxis->setLabel("y");
    customPlot7->axisRect()->setupFullAxesBox(true);
    customPlot7->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    customPlot7->graph(0)->setLineStyle(QCPGraph::lsNone);
    customPlot7->graph(1)->setLineStyle(QCPGraph::lsNone);
    customPlot7->graph(2)->setLineStyle(QCPGraph::lsNone);
    customPlot7->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
    customPlot7->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
    customPlot7->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));

    //Customplot8
    customPlot8->addGraph();
    customPlot8->graph(0)->setPen(QPen(Qt::red));
    customPlot8->addGraph();
    customPlot8->graph(1)->setPen(QPen(Qt::blue));
    customPlot8->addGraph();
    customPlot8->graph(2)->setPen(QPen(Qt::green));
    customPlot8->addGraph();
    customPlot8->graph(3)->setPen(QPen(Qt::yellow));
    customPlot8->addGraph();
    customPlot8->graph(4)->setPen(QPen(Qt::black));
    customPlot8->xAxis->setTicker(timeTicker);
    customPlot8->yAxis->setRange(0, 1000);
    customPlot8->xAxis->setLabel("time (s)");
    customPlot8->yAxis->setLabel("angular velocity (rad/s)");    
    customPlot8->axisRect()->setupFullAxesBox(true);
    customPlot8->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    //Customplot9
    customPlot9->addGraph();
    customPlot9->graph(0)->setPen(QPen(Qt::red));
    customPlot9->addGraph();
    customPlot9->graph(1)->setPen(QPen(Qt::blue));
    customPlot9->addGraph();
    customPlot9->graph(2)->setPen(QPen(Qt::green));
    customPlot9->addGraph();
    customPlot9->graph(3)->setPen(QPen(Qt::black));
    customPlot9->addGraph();
    customPlot9->graph(4)->setPen(QPen(Qt::yellow));
    customPlot9->xAxis->setTicker(timeTicker);
    customPlot9->yAxis->setRange(0, 10);
    customPlot9->xAxis->setLabel("time (s)");
    customPlot9->yAxis->setLabel("current (A)");
    customPlot9->axisRect()->setupFullAxesBox(true);
    customPlot9->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    //Customplot10
    customPlot10->addGraph();
    customPlot10->graph(0)->setPen(QPen(Qt::red));
    customPlot10->addGraph();
    customPlot10->graph(1)->setPen(QPen(Qt::blue));
    customPlot10->addGraph();
    customPlot10->graph(2)->setPen(QPen(Qt::green));
    customPlot10->xAxis->setTicker(timeTicker);
    customPlot10->yAxis->setRange(-3, 3);
    customPlot10->xAxis->setLabel("time (s)");
    customPlot10->yAxis->setLabel("rotational rate (deg/s)");
    customPlot10->axisRect()->setupFullAxesBox(true);
    customPlot10->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    //Customplot11
    customPlot11->addGraph();
    customPlot11->graph(0)->setPen(QPen(Qt::red));
    customPlot11->addGraph();
    customPlot11->graph(1)->setPen(QPen(Qt::blue));
    customPlot11->addGraph();
    customPlot11->graph(2)->setPen(QPen(Qt::green));
//    customPlot11->addGraph();
//    customPlot11->graph(3)->setPen(QPen(Qt::black));
    customPlot11->xAxis->setTicker(timeTicker);
    customPlot11->yAxis->setRange(-1.5, 1.5);
    customPlot11->xAxis->setLabel("time (s)");
    customPlot11->yAxis->setLabel("accelerometer (m/s/s)");
    customPlot11->axisRect()->setupFullAxesBox(true);
    customPlot11->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    //Customplot12
    customPlot12->addGraph();
    customPlot12->graph(0)->setPen(QPen(Qt::red));
//    customPlot12->xAxis->setTicker(timeTicker);
    customPlot12->yAxis->setRange(-2, 2);
    customPlot12->xAxis->setRange(-2, 2);
    customPlot12->xAxis->setLabel("y (m)");
    customPlot12->yAxis->setLabel("x (m)");
    customPlot12->axisRect()->setupFullAxesBox(true);
    customPlot12->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    //Customplot13
    customPlot13->addGraph();
    customPlot13->graph(0)->setPen(QPen(Qt::red));
    customPlot13->addGraph();
    customPlot13->graph(1)->setPen(QPen(Qt::blue));
    customPlot13->addGraph();
    customPlot13->graph(2)->setPen(QPen(Qt::green));
    customPlot13->addGraph();
//    customPlot13->graph(3)->setPen(QPen(Qt::cyan));
    customPlot13->xAxis->setTicker(timeTicker);
    customPlot13->yAxis->setRange(-20, 20);
    customPlot13->xAxis->setLabel("time (s)");
    customPlot13->yAxis->setLabel("differential timestamp (msec)");
    customPlot13->axisRect()->setupFullAxesBox(true);
    customPlot13->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}
void MainWindow::plotGraphs()
{
    if( !ui->checkBox_12->isChecked()  ){ return; }

    static QTime time(QTime::currentTime());
    double key = time.elapsed()/1000.0;
    double RANGE = 3;

    if( ui->tabWidget->currentIndex() == 0 )
    {
        //CAMERA POSITION AND QUATERNION
        //CustomPlot1
        ui->customPlot1->graph(0)->addData(key, cameraEKF::x(0)*100);
        ui->customPlot1->graph(1)->addData(key, cameraEKF::x(1)*100);
        ui->customPlot1->graph(2)->addData(key, cameraEKF::x(2)*100);
//        ui->customPlot1->yAxis->rescale();
        ui->customPlot1->xAxis->setRange(key, RANGE, Qt::AlignRight);
        ui->customPlot1->replot();

        //CustomPlot2
        vec neta = quat2angle(attitudeControl::quaternion);

        ui->customPlot2->graph(0)->addData(key, neta(0));
        ui->customPlot2->graph(1)->addData(key, neta(1));
        ui->customPlot2->graph(2)->addData(key, neta(2));
//        ui->customPlot2->yAxis->rescale();
        ui->customPlot2->xAxis->setRange(key, RANGE, Qt::AlignRight);
        ui->customPlot2->replot();
    }
    else if( ui->tabWidget->currentIndex() == 1 )
    {
        //Customplot12
        ui->customPlot12->removeGraph(1);
//        ui->customPlot12->addGraph();
        ui->customPlot12->graph(0)->setPen(QPen(Qt::red));
        ui->customPlot12->axisRect()->setupFullAxesBox();
        ui->customPlot12->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        ui->customPlot12->graph(0)->setLineStyle(QCPGraph::lsImpulse);

        ui->customPlot12->addGraph();
        ui->customPlot12->graph(1)->setPen(QPen(Qt::red));
        ui->customPlot12->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));

        if( motionCapture::ekfType == EKF_MULTI_BLOB )
        {
//            ui->customPlot12->graph(0)->addData(cameraEKF::x[0], cameraEKF::x[1]);
            ui->customPlot12->graph(1)->addData(-cameraEKF::x[1], cameraEKF::x[0]);
        }
        else
        {
//            ui->customPlot12->graph(0)->addData(ekfSingleBlob::x[0], ekfSingleBlob::x[1]);
            ui->customPlot12->graph(1)->addData(-ekfSingleBlob::x[1], ekfSingleBlob::x[0]);
        }


        ui->customPlot12->replot();


    }
    else if( ui->tabWidget->currentIndex() == 2 )
    {
        QVector<double> x(2),y(2);
        ui->customPlot1->graph(0)->setData(x,y);
        ui->customPlot1->graph(1)->setData(x,y);
        ui->customPlot1->graph(2)->setData(x,y);
        ui->customPlot2->graph(0)->setData(x,y);
        ui->customPlot2->graph(1)->setData(x,y);
        ui->customPlot2->graph(2)->setData(x,y);

        ui->customPlot3->clearGraphs();
        ui->customPlot4->clearGraphs();
        ui->customPlot5->clearGraphs();
        ui->customPlot6->clearGraphs();
        ui->customPlot7->clearGraphs();

        //Customplot3
        ui->customPlot3->addGraph();
        ui->customPlot3->graph(0)->setPen(QPen(Qt::red));
        ui->customPlot3->addGraph();
        ui->customPlot3->graph(1)->setPen(QPen(Qt::blue));
        ui->customPlot3->addGraph();
        ui->customPlot3->graph(2)->setPen(QPen(Qt::green));
        ui->customPlot3->axisRect()->setupFullAxesBox();
        ui->customPlot3->yAxis->setRange(0, 200);
        ui->customPlot3->xAxis->setRange(0, 320);
        ui->customPlot3->xAxis->setLabel("x");
        ui->customPlot3->yAxis->setLabel("y");
        ui->customPlot3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        ui->customPlot3->graph(0)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot3->graph(1)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot3->graph(2)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot3->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
        ui->customPlot3->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
        ui->customPlot3->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));


        //Customplot4
        ui->customPlot4->addGraph();
        ui->customPlot4->graph(0)->setPen(QPen(Qt::red));
        ui->customPlot4->addGraph();
        ui->customPlot4->graph(1)->setPen(QPen(Qt::blue));
        ui->customPlot4->addGraph();
        ui->customPlot4->graph(2)->setPen(QPen(Qt::green));
        ui->customPlot4->axisRect()->setupFullAxesBox();
        ui->customPlot4->yAxis->setRange(0, 200);
        ui->customPlot4->xAxis->setRange(0, 320);
        ui->customPlot4->xAxis->setLabel("x");
        ui->customPlot4->yAxis->setLabel("y");
        ui->customPlot4->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        ui->customPlot4->graph(0)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot4->graph(1)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot4->graph(2)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot4->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
        ui->customPlot4->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
        ui->customPlot4->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));

        //Customplot5
        ui->customPlot5->addGraph();
        ui->customPlot5->graph(0)->setPen(QPen(Qt::red));
        ui->customPlot5->addGraph();
        ui->customPlot5->graph(1)->setPen(QPen(Qt::blue));
        ui->customPlot5->addGraph();
        ui->customPlot5->graph(2)->setPen(QPen(Qt::green));
        ui->customPlot5->axisRect()->setupFullAxesBox();
        ui->customPlot5->yAxis->setRange(0, 200);
        ui->customPlot5->xAxis->setRange(0, 320);
        ui->customPlot5->xAxis->setLabel("x");
        ui->customPlot5->yAxis->setLabel("y");
        ui->customPlot5->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        ui->customPlot5->graph(0)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot5->graph(1)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot5->graph(2)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot5->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
        ui->customPlot5->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
        ui->customPlot5->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));

        //Customplot6
        ui->customPlot6->addGraph();
        ui->customPlot6->graph(0)->setPen(QPen(Qt::red));
        ui->customPlot6->addGraph();
        ui->customPlot6->graph(1)->setPen(QPen(Qt::blue));
        ui->customPlot6->addGraph();
        ui->customPlot6->graph(2)->setPen(QPen(Qt::green));
        ui->customPlot6->axisRect()->setupFullAxesBox();
        ui->customPlot6->yAxis->setRange(0, 200);
        ui->customPlot6->xAxis->setRange(0, 320);
        ui->customPlot6->xAxis->setLabel("x");
        ui->customPlot6->yAxis->setLabel("y");
        ui->customPlot6->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        ui->customPlot6->graph(0)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot6->graph(1)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot6->graph(2)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot6->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
        ui->customPlot6->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
        ui->customPlot6->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));

        //Customplot7
        ui->customPlot7->addGraph();
        ui->customPlot7->graph(0)->setPen(QPen(Qt::red));
        ui->customPlot7->addGraph();
        ui->customPlot7->graph(1)->setPen(QPen(Qt::blue));
        ui->customPlot7->addGraph();
        ui->customPlot7->graph(2)->setPen(QPen(Qt::green));
        ui->customPlot7->axisRect()->setupFullAxesBox();
        ui->customPlot7->yAxis->setRange(0, 200);
        ui->customPlot7->xAxis->setRange(0, 320);
        ui->customPlot7->xAxis->setLabel("x");
        ui->customPlot7->yAxis->setLabel("y");
        ui->customPlot7->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
        ui->customPlot7->graph(0)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot7->graph(1)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot7->graph(2)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot7->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
        ui->customPlot7->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
        ui->customPlot7->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));

//        //RAW CAMERA DATA
        //CustomPlot3
        for(int i=0;i<3;i++)
        {
            if(i == motionCapture::fDet(0))
            {
                break;
            }
            ui->customPlot3->graph(i)->addData(motionCapture::Bp(0,i,0), motionCapture::Bp(1,i,0));
        }
        ui->customPlot3->replot();
        //CustomPlot4
        for(int i=0;i<3;i++)
        {
            if(i == motionCapture::fDet(1))
            {
                break;
            }
            ui->customPlot4->graph(i)->addData(motionCapture::Bp(0,i,1), motionCapture::Bp(1,i,1));
        }
        ui->customPlot4->replot();
        //CustomPlot5
        for(int i=0;i<3;i++)
        {
            if(i == motionCapture::fDet(2))
            {
                break;
            }
            ui->customPlot5->graph(i)->addData(motionCapture::Bp(0,i,2), motionCapture::Bp(1,i,2));
        }
        ui->customPlot5->replot();
        //CustomPlot6
        for(int i=0;i<3;i++)
        {
            if(i == motionCapture::fDet(3))
            {
                break;
            }
            ui->customPlot6->graph(i)->addData(motionCapture::Bp(0,i,3), motionCapture::Bp(1,i,3));
        }
        ui->customPlot6->replot();
        //CustomPlot7
        for(int i=0;i<3;i++)
        {
            if(i == motionCapture::fDet(4))
            {
                break;
            }
            ui->customPlot7->graph(i)->addData(motionCapture::Bp(0,i,4), motionCapture::Bp(1,i,4));
        }
        ui->customPlot7->replot();
    }
    else if( ui->tabWidget->currentIndex() == 3 )
    {
        //MOTOR SPEEDS AND CURRENTS
        //CustomPlot8
        ui->customPlot8->graph(0)->addData(key, motorSpeedControl::motorSpeed[0]);
        ui->customPlot8->graph(1)->addData(key, motorSpeedControl::motorSpeed[1]);
        ui->customPlot8->graph(2)->addData(key, motorSpeedControl::motorSpeed[2]);
        ui->customPlot8->graph(3)->addData(key, motorSpeedControl::motorSpeed[3]);
        ui->customPlot8->graph(4)->addData(key, motorSpeedControl::reference);
//        ui->customPlot8->yAxis->rescale();
        ui->customPlot8->xAxis->setRange(key, RANGE, Qt::AlignRight);
        ui->customPlot8->replot();

        //CustomPlot9
        ui->customPlot9->graph(0)->addData(key, motorSpeedControl::current[0]);
        ui->customPlot9->graph(1)->addData(key, motorSpeedControl::current[1]);
        ui->customPlot9->graph(2)->addData(key, motorSpeedControl::current[2]);
        ui->customPlot9->graph(3)->addData(key, motorSpeedControl::current[3]);
//        ui->customPlot9->yAxis->rescale();
        ui->customPlot9->xAxis->setRange(key, RANGE, Qt::AlignRight);
        ui->customPlot9->replot();
    }
    else if( ui->tabWidget->currentIndex() == 4 )
    {
        //IMU DATA
        //CustomPlot10
        ui->customPlot10->graph(0)->addData(key, imu::gyro[0]*180/M_PI);
        ui->customPlot10->graph(1)->addData(key, imu::gyro[1]*180/M_PI);
        ui->customPlot10->graph(2)->addData(key, imu::gyro[2]*180/M_PI);
//        ui->customPlot10->yAxis->rescale();
        ui->customPlot10->xAxis->setRange(key, RANGE, Qt::AlignRight);
        ui->customPlot10->replot();

        //CustomPlot11
        ui->customPlot11->graph(0)->addData(key, imu::accel[0]);
        ui->customPlot11->graph(1)->addData(key, imu::accel[1]);
        ui->customPlot11->graph(2)->addData(key, imu::accel[2]);

//        ui->customPlot11->graph(3)->addData(key, imu::accelNotched[2]);
//        ui->customPlot11->yAxis->rescale();
        ui->customPlot11->xAxis->setRange(key, RANGE, Qt::AlignRight);
        ui->customPlot11->replot();
    }
    else
    {
        //COMMS
        //CustomPlot13
        ui->customPlot13->graph(0)->addData(key, serialWixel::dT);
        ui->customPlot13->graph(1)->addData(key, serialRadio::dT);
        ui->customPlot13->graph(2)->addData(key, cameraEKF::dT*1e3);
//        ui->customPlot10->yAxis->rescale();
        ui->customPlot13->xAxis->setRange(key, RANGE, Qt::AlignRight);
        ui->customPlot13->replot();
    }
}

void MainWindow::on_checkBox_clicked(bool checked)
{
    dataLogger::toggle(checked);
}

void MainWindow::on_pushButton_clicked()
{
    QApplication::quit();
}

void MainWindow::on_checkBox_6_clicked(bool checked)
{
    calibrationMode::isCalibrating = checked;
}

void MainWindow::on_checkBox_2_clicked(bool checked)
{
    motorSpeedControl::isClosedLoop = checked;
}

void MainWindow::on_pushButton_3_clicked()
{
    stateMachine::resetMicro = true;
}

void MainWindow::on_pushButton_4_clicked()
{
    qDebug() << '\r' << '\r' << '\r' << '\r' << '\r' << '\r' << '\r' << '\r' << '\r' << '\r' << '\r' << '\r' << '\r' << '\r' << '\r' << '\r';
}

void MainWindow::on_checkBox_10_clicked()
{
    cameraEKF::initialise();
}

void MainWindow::on_checkBox_14_clicked(bool checked)
{
    cameraEKF::cameraDataOnly = checked;
}

void MainWindow::on_pushButton_6_clicked()
{
    stateMachine::thrust_0 = imu::gyro[0];
}
