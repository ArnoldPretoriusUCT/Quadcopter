#include "imu.h"

double imu::gyro[3],
       imu::accel[3],
       imu::accelNotched[3],
       imu::temperature;

bool imu::newData;

