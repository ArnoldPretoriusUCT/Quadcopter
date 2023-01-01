#ifndef COMMON_H

#define COMMON_H

#include <armadillo>
#include <iostream>
#include <QtCore/QDebug>
#include <QSignalMapper>
#include <QElapsedTimer>

using namespace arma;
using namespace std;

vec quat2angle(vec quat);
mat quat2rotationMatrix(vec quat);
mat euler2quaternion(vec angle);
vec quatMultiply(vec q1, vec q2);
vec quatConjugate(vec q);


void double2chars(double DOUBLE, const double scalar, char CHAR[2]);
double chars2double(QByteArray buffer,unsigned char index,const double scalar);
double uchars2double(QByteArray buffer,unsigned char index,const double scalar);

double uchar2double(QByteArray buffer, unsigned char index, const double scalar);
double char2double(QByteArray buffer, unsigned char index, const double scalar);

static QElapsedTimer elapsedTimer;

#endif // COMMON_H

