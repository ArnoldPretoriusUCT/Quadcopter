#ifndef _floki_H_
#define _floki_H_

#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Metro.h"
#include "common.h"
#include "PWM.h"
#include "RATECONTROL.h"
#include "RADIO.h"
#include "MOTORSPEEDCONTROL.h"
#include "HEAVECONTROL.h"
#include "IMU.h"
#include "WIXEL.h"
#include "WATCHDOG.h"
#include "LED.h"
#include "ECF.h"
#include "ENCODER.h"

#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif





#endif /* _floki_H_ */
