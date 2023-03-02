/*
 * common.h
 *
 *  Created on: 02 Dec 2015
 *      Author: Arnold
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "stdlib.h"
#include "math.h"
#include "Arduino.h"

#define dT 0.005
#define g 9.8
#define RPM_TO_RADPS (2*M_PI/60)
#define LED_PIN 13
#define IR_LED_PIN 12
//#define RADIO_BUS Serial1
//#define XBEE_BUS Serial3

float LPF(const float data, const float dt, const float w, float state[2]);
float LPF2(const float data, const float dt, float w, float ux[2], float yx[2]);
float LPF_UD(const float data, const float dt, const float wn, const float z, const float a, float state[6]);
float HPF(const float data, const float dt, const float w, float state[2]);
float HPF2(const float data, const float dt, float w, float ux[3], float yx[3]);
float digitalNotchFilter(const float data, const float dt, const float w, const float Q, float ux[2], float yx[2]);
float PD(const float e, const float K, const float wz, const float wp, const float dt, float state[2]);
float PIc(const float e, const float K, const float w_zero, const float dt, const float u_max, float state[3]);
float PID(const float e0, const float K, const float w_zero1, const float w_zero2, const float w_pole, const float dt, float state[4]);
void complementaryFilter(const float u, const float w, float state[4], float y[2]);
float chars2float(unsigned char CHAR[2],const float scalar);
float uchars2float(unsigned char CHAR[2],const float scalar);
void float2chars(const float FLOAT,const float scalar,unsigned char CHAR[2]);
unsigned char float2UINT8(const float FLOAT, const float scalar);
signed char float2INT8(const float FLOAT, const float scalar);
float magicFilter(const float u, float *u_past);
float sineInput(const float amp, const float freq_radps, const float timePoint);
float sign(const float u);
void cross(const float a[3], const float b[3], float ab[3]);
void getDerivative(const float x[3], float x_past[3], const float dt, float dx[3]);
void invert3by3Matrix(const float A[9], float invA[9]);
void scaleArray(const float* A, const int sizeA, const float k, float* B);

void rotateAboutZAxis(const float theta, float x[2]);

uint16_t getDT(uint32_t * t_past);

void printArray(const float array[], const uint8_t array_size);

#endif /* COMMON_H_ */
