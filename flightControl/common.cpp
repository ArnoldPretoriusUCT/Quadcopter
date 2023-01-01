#include "common.h"

vec quat2angle(vec quat)
{
    double q0 = quat(0);
    double q1 = quat(1);
    double q2 = quat(2);
    double q3 = quat(3);

    vec neta(3);
    neta(0) = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))*180/M_PI;
    neta(1) = asin(-2*(q1*q3-q0*q2))*180/M_PI;
    neta(2) = atan2(2*(q1*q2+q0*q3),1-2*(q2*q2+q3*q3))*180/M_PI;

    return neta;
}

mat quat2rotationMatrix(vec quat)
{
    double q0 = quat(0);
    double q1 = quat(1);
    double q2 = quat(2);
    double q3 = quat(3);

    mat R(3,3);
    R  << 1-2*(q2*q2+q3*q3) << 2*(q1*q2-q0*q3)   << 2*(q1*q3+q0*q2)  << endr
       <<  2*(q1*q2+q0*q3)  << 1-2*(q1*q1+q3*q3) << 2*(q2*q3-q0*q1)   << endr
       <<  2*(q1*q3-q0*q2)  <<  2*(q2*q3+q0*q1)  << 1-2*(q1*q1+q2*q2) << endr;

    return R;
}

mat euler2quaternion(vec angle)
{
    double phi=angle(0);
    double the = angle(1);
    double psi = angle(2);

    vec q;
    q << cos(phi/2)*cos(the/2)*cos(psi/2)+sin(phi/2)*sin(the/2)*sin(psi/2)
       << sin(phi/2)*cos(the/2)*cos(psi/2)-cos(phi/2)*sin(the/2)*sin(psi/2)
       << cos(phi/2)*sin(the/2)*cos(psi/2)+sin(phi/2)*cos(the/2)*sin(psi/2)
       << cos(phi/2)*cos(the/2)*sin(psi/2)-sin(phi/2)*sin(the/2)*cos(psi/2) << endr;

    return q;
}

vec quatMultiply(vec q, vec p)
{
    vec qOut;
    qOut << q(0)*p(0)-q(1)*p(1)-q(2)*p(2)-q(3)*p(3)
         << q(0)*p(1)+q(1)*p(0)+q(2)*p(3)-q(3)*p(2)
         << q(0)*p(2)-q(1)*p(3)+q(2)*p(0)+q(3)*p(1)
         << q(0)*p(3)+q(1)*p(2)-q(2)*p(1)+q(3)*p(0)
         << endr;

    return qOut;
}

vec quatConjugate(vec q)
{
    vec qOut;
    qOut << q(0) << -q(1) << -q(2) << -q(3) << endr;

    return qOut;
}

void double2chars(double DOUBLE, const double scaler, char CHAR[2])
{
    qint16 INT = (qint16)(scaler*DOUBLE);
    CHAR[0] = (char)( INT >> 8 );
    CHAR[1] = (char)( INT & 0xFF );
}

double chars2double(QByteArray buffer,unsigned char index,const double scaler)
{
    qint16 msb = ((qint16)buffer[index] << 8) & 0xFF00;
    quint8 lsb = (quint8)buffer[index+1];

    double out = (double)( msb | lsb )/scaler;
    return out;
}

double uchars2double(QByteArray buffer,unsigned char index,const double scaler)
{
    quint16 msb = ((quint16)buffer[index] << 8) & 0xFF00;
    quint8 lsb = (quint8)buffer[index+1];

    double out = (double)( msb | lsb )/scaler;
    return out;
}

double uchar2double(QByteArray buffer, unsigned char index, const double scalar)
{
    double out = (double)(scalar*(quint8)buffer[index]);
    return out;
}

double char2double(QByteArray buffer, unsigned char index, const double scalar)
{
    double out = (double)(scalar*(qint8)buffer[index]);
    return out;
}
