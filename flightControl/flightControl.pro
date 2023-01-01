#-------------------------------------------------
#
# Project created by QtCreator 2016-11-22T13:39:50
#
#-------------------------------------------------

QT       += core gui
QT       += serialport

#QMAKE_CXXFLAGS_RELEASE *= -O3
QMAKE_CXXFLAGS_RELEASE *= -Ofast
QMAKE_CXXFLAGS += -std=c++11

DEFINES += QCUSTOMPLOT_USE_OPENGL

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = flightControl
TEMPLATE = app

LIBS += -llapack -lblas -larmadillo

SOURCES += main.cpp\
        mainwindow.cpp \
    common.cpp \
    motioncapture.cpp \
    qcustomplot.cpp \
    serialPixy.cpp \
    cameraEKF.cpp \
    statemachine.cpp \
    attitudecontrol.cpp \
    motorspeedcontrol.cpp \
    imu.cpp \
    datalogger.cpp \
    serialradio.cpp \
    serialwixel.cpp \
    positioncontrol.cpp \
    calibrationmode.cpp \
    ekfsingleblob.cpp

HEADERS  += mainwindow.h \
    common.h \
    motioncapture.h \
    qcustomplot.h \
    serialPixy.h \
    cameraEKF.h \
    statemachine.h \
    attitudecontrol.h \
    motorspeedcontrol.h \
    imu.h \
    datalogger.h \
    serialradio.h \
    serialwixel.h \
    positioncontrol.h \
    calibrationmode.h \
    ekfsingleblob.h

FORMS    += mainwindow.ui
