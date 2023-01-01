#ifndef IMU_H
#define IMU_H


class imu
{
    public:
        static double gyro[3],accel[3],accelNotched[3],temperature;
        static bool newData;
};

#endif // IMU_H
