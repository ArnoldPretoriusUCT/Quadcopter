/*
 * common.cpp
 *
 *  Created on: 02 Dec 2015
 *      Author: Arnold
 */
#include "common.h"

//data is the input
//dt is the sampling time in seconds
//w is frequency in rad/s
//state stores the past input and output signal state = {u[1] y[1]}

float LPF(const float data, const float dt, const float w, float state[2])
{
	static float y[2];
	static float u[2];

	float a = 1 + 2/(w*dt);
	float b = 2/(w*dt)-1;

	u[0] = data;
	u[1] = state[0];
	y[1] = state[1];

	y[0] = b/a * y[1] + 1/a * (u[0] + u[1]);
	y[1] = y[0];
	u[1] = u[0];

	state[0] = u[0];
	state[1] = y[0];

	return y[0];
}

float LPF2(const float data, const float dt, float w, float ux[2], float yx[2])
{
	static float y[3];
	static float u[3];

	float a = 1 + 4/(w*dt) + 4/( pow(w,2)*pow(dt,2) );
	float b = 8/( pow(w,2)*pow(dt,2) ) - 2 ;
	float c = 4/(w*dt) - 1 - 4/( pow(w,2)*pow(dt,2) );

	u[0] = data;
	u[1] = ux[0];
	u[2] = ux[1];
	y[1] = yx[0];
	y[2] = yx[1];

	y[0] = b/a * y[1] + c/a * y[2] + 1/a * (u[0] + 2*u[1] + u[2]);
	yx[1] = y[1];
	yx[0] = y[0];
	ux[1] = u[1];
	ux[0] = u[0];

	return y[0];
}

float LPF_UD(const float data, const float dt, const float wn, const float z, const float a, float state[5])
{
	float a0 = 4*dt*wn;
	float a1 = dt*dt*wn*wn+4;
	float a2 = 2*dt*dt*wn*wn-4*dt*wn-8;
	float a3 = dt*dt*wn*wn+4;
	
	float b1 = a*dt*dt*wn*wn;
	float b2 = 2*b1;
	float b3 = b1;
	
	static float y[4];
	static float u[3];
	
	u[0] = data;
	u[1] = state[0];
	u[2] = state[1];
	y[1] = state[2];
	y[2] = state[3];
	y[3] = state[4];
	
	y[0] = 1/a0*( b1*u[0]+b2*u[1]+b3*u[2]-a1*y[1]-a2*y[2]-a3*y[3] );
	state[0] = u[0];
	state[1] = u[1];
	state[2] = y[0];
	state[3] = y[1];
	state[4] = y[2];
	
	return y[0];
}

float HPF(const float data, const float dt, const float w, float state[2])
{
	static float y[] = {0,0};
	static float u[] = {0,0};

	static char cnt = 0;

	float a = 1 + 2/(w*dt);
	float b = 2/(w*dt) - 1;
	float c = 2/(w*dt);

	u[0] = data;
	u[1] = state[0];
	y[1] = state[1];

	y[0] = b/a * y[1] + c/a * (u[0] - u[1]);
	y[1] = y[0];
	u[1] = u[0];

	state[0] = u[0];
	state[1] = y[0];

	return y[0];
}

float HPF2(const float data, const float dt, float w, float ux[3], float yx[3])
{
	static float y[3];// = {ic,ic};
	static float u[3];// = {ic,ic};

	float a = 1 + 4/(w*dt) + 4/( pow(w,2)*pow(dt,2) );
	float b = 8/( pow(w,2)*pow(dt,2) ) - 2;
	float c = 4/(w*dt) - 1 - 4/( pow(w,2)*pow(dt,2) );
	float d = 4/( pow(w,2)*pow(dt,2) ) + 4/(w*dt);
	float e = -8/( pow(w,2)*pow(dt,2) );
	float f = 4/( pow(w,2)*pow(dt,2) ) - 4/(w*dt);

	u[0] = data;
	u[1] = ux[1];
	u[2] = ux[2];
	y[0] = yx[0];
	y[1] = yx[1];
	y[2] = yx[2];

	y[0] = b/a * y[1] + c/a * y[2] + d/a * u[0] + e/a * u[1] + f/a * u[2];
	y[2] = y[1];
	y[1] = y[0];
	u[2] = u[1];
	u[1] = u[0];

	ux[0] = u[0];
	ux[1] = u[1];
	ux[2] = u[2];
	yx[0] = y[0];
	yx[1] = y[1];
	yx[2] = y[2];

	return y[0];
}

float digitalNotchFilter(const float data, const float dt, const float w, const float Q, float ux[2], float yx[2])
{
	static float y[3];// = {ic,ic};
	static float u[3];// = {ic,ic};

	float a = Q*dt*dt*w*w+4+4*dt*w;
	float b = 2*Q*dt*dt*w*w-8;
	float c = Q*dt*dt*w*w+4-4*dt*w;
	float d = Q*dt*dt*w*w+4+2*dt*w;
	float e = 2*Q*dt*dt*w*w-8;
	float f = Q*dt*dt*w*w+4-2*dt*w;

	u[0] = data;
	u[1] = ux[0];
	u[2] = ux[1];
	y[1] = yx[0];
	y[2] = yx[1];

	y[0] = -b/a * y[1] - c/a * y[2] + d/a * u[0] + e/a * u[1] + f/a * u[2];

	yx[1] = y[1];
	yx[0] = y[0];
	ux[1] = u[1];
	ux[0] = u[0];

	return y[0];
}

float PD(const float e0, const float K, const float wz, const float wp, const float dt, float state[2])
{
	float a0 = 2+dt*wp;
	float a1 = dt*wp-2;
	float b0 = K*wp*(2/wz+dt);
	float b1 = K*wp*(dt-2/wz);

	float u = state[0];
	float e1 = state[1];

	u = 1/a0*( -a1*u + b0*e0 + b1*e1 );
	state[0] = u;
	state[1] = e0;

	return u;
}
//G=K(s+w_zero)/s
float PIc(const float e, const float K, const float w_zero, const float dt, const float u_max, float state[3])
{
	float u_sat,b;

	b = LPF(state[0], dt, w_zero, &state[1]);
	u_sat = e + b;

	if( fabs(u_sat) >= u_max/K )
	{
		u_sat = sign(u_sat)*u_max/K;
	}
	float u = K*u_sat;

	state[2] = b;
	state[1] = state[0];
	state[0] = u_sat;

	return u;
}
//G=K(1+s/w_zer1)(1+s/w_zero2)/s/(1+s/w_pole)
float PID(const float e, const float K, const float w_zero1, const float w_zero2, const float w_pole, const float dt, float state[3])
{
	float Td=1.0/w_zero1;
	float aTd = 1.0/w_pole;
	float Ti=1.0/w_zero2;
	float kp = K*Ti;

	float e_= state[0];
	float x_ = state[1];
	float u_ = state[2];

	float x = (aTd*x_+(Td+dT)*e-Td*e_)/(aTd+dT);
	float dp = kp*(x-x_);
	float ds = kp*dT/Ti*e;
	float du = dp+ds;
	float u = u_+du;

	state[0] = e;
	state[1] = x;
	state[2] = u;

	return u;
}

void complementaryFilter(const float u, const float w, float state[4], float y[2])
{
	y[0] = LPF(u, dT, w, &state[0]);
	y[1] = HPF(u, dT, w, &state[2]);
}

float sineInput(const float amp, const float freq_radps,float timePoint)
{
	float theta = amp*sin(freq_radps*timePoint);
	return theta;
}

float chars2float(unsigned char CHAR[2],const float scalar)
{
	int16_t INT_MSB = ((int16_t)CHAR[0] << 8) & 0xFF00;
	uint8_t INT_LSB = (uint8_t)CHAR[1];
	float FLOAT = (float)( INT_MSB | INT_LSB )/scalar;
	return FLOAT;
}

float uchars2float(unsigned char CHAR[2],const float scalar)
{
	uint16_t INT_MSB = ((uint16_t)CHAR[0] << 8) & 0xFF00;
	uint8_t INT_LSB = (uint8_t)CHAR[1];
	float FLOAT = (float)( INT_MSB | INT_LSB )/scalar;
	return FLOAT;
}

void float2chars(const float FLOAT,const float scaler,unsigned char CHAR[2])
{
	int16_t INT = (int16_t)(scaler*FLOAT);
	CHAR[0] = (unsigned char)( INT >> 8 );
	CHAR[1] = (unsigned char)( INT & 0xFF );
}

unsigned char float2UINT8(const float FLOAT, const float scalar)
{
	unsigned char UINT8 = (unsigned char)(scalar*FLOAT);
	return UINT8;
}

signed char float2INT8(const float FLOAT, const float scalar)
{
	signed char INT8 = (signed char)(scalar*FLOAT);
	return INT8;
}

float magicFilter(const float u, float *u_past)
{
	float uDev = .001;	//3cm
	float y;

	if( abs(u-*u_past) > uDev )
	{
		y = *u_past;
	}
	else
	{
		y = u;
	}
	*u_past = y;
	return y;
}

float sign(const float u)
{
	if( u > 0 )
	{
		return 1;
	}
	else if( u < 0 )
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

void cross(const float a[3], const float b[3], float ab[3])
{
	ab[0] = a[1]*b[2] - a[2]*b[1];
	ab[1] = a[2]*b[0] - a[0]*b[2];
	ab[2] = a[0]*b[1] - a[1]*b[0];
}

void getDerivative(const float x[3], float x_past[3], const float dt, float dx[3])
{
	dx[0] = (x[0] - x_past[0])/dt;
	x_past[0] = x[0];

	dx[1] = (x[1] - x_past[1])/dt;
	x_past[1] = x[1];

	dx[2] = (x[2] - x_past[2])/dt;
	x_past[2] = x[2];
}

void invert3by3Matrix(const float A[9], float invA[9])
{
	float Ct[] = {A[4]*A[8]-A[7]*A[5], A[2]*A[7]-A[1]*A[8], A[1]*A[5]-A[2]*A[4],
				 A[5]*A[6]-A[3]*A[8], A[0]*A[8]-A[2]*A[6], A[2]*A[3]-A[0]*A[5],
				 A[3]*A[7]-A[4]*A[6], A[1]*A[6]-A[0]*A[7], A[0]*A[4]-A[1]*A[3]};


	float detA = A[0]*A[4]*A[8]+A[1]*A[5]*A[6]+A[2]*A[3]*A[7] -A[2]*A[4]*A[6]-A[0]*A[5]*A[7]-A[1]*A[3]*A[8];
	for(int i=0;i<9;i++)
	{
		if(detA == 0)
		{
			Serial.println("Inversion failed: Matrix singular :(");
		}
		else
		{
			invA[i] = Ct[i]/detA;
		}

	}
//	Serial.println(detA*1e25);
}

void scaleArray(const float* A, const int sizeA, const float k, float* B)
{
	for(int i=0;i<sizeA;i++)
	{
		B[i] = k*A[i];
	}
}
//Writes result to input matrix
//Anti-clockwise?
void rotateAboutZAxis(const float theta, float x[2])
{
	float temp[2];
	temp[0] =  cos(theta)*x[0] + sin(theta)*x[1];
	temp[1] = -sin(theta)*x[0] + cos(theta)*x[1];

	x[0] = temp[0];
	x[1] = temp[1];
}

void printArray(const float array[], const uint8_t array_size)
{
	for(int i=0;i<array_size;i++)
	{
		Serial.print(array[i]);
		Serial.print(", ");
	}
	Serial.print('\n');
}

uint16_t getDT(uint32_t * t_past)
{
	static bool start = 0;

	uint32_t dt = micros() - *t_past;
	*t_past += dt;

	if( start == 0 )
	{
		start = 1;
		return 0;

	}
	return dt;
}
