/*
 * ADNS3080.cpp
 *
 *  Created on: 07 Mar 2016
 *      Author: Arnold
 */

#include "ADNS3080.h"

signed char ADNS3080::dx_cnt,ADNS3080::dy_cnt;
uint8_t ADNS3080::surfaceQuality;
float ADNS3080::velocity[2];

ADNS3080::ADNS3080()
{
	dx_cnt = dy_cnt = 0;
	velocity[0] = velocity[1] = 0;
	resolution = ledAlwaysOn = 0;
	temp = 0;
	surfaceQuality = 0;
}

void ADNS3080::initialise()
{
	pinMode( ADNS3080_CHIP_SELECT,OUTPUT );
	pinMode( ADNS3080_RESET,OUTPUT );

	digitalWrite(ADNS3080_RESET,HIGH);
	delay(1);
	digitalWrite(ADNS3080_RESET,LOW);

	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE3);
	SPI.setClockDivider(SPI_CLOCK_DIV8);

	temp = read(ADNS3080_PRODUCT_ID);
	temp = read(ADNS3080_PIXEL_SUM);
	temp = read(ADNS3080_MOTION);
	temp = read(ADNS3080_CONFIGURATION_BITS);

	setResolution(ADNS3080_RESOLUTION1600);
}

signed char ADNS3080::read(unsigned char address)
{
  signed char data;
  // take the CS pin low to select the chip:
  digitalWrite(ADNS3080_CHIP_SELECT,LOW);
  //  send in the address and value via SPI:
  SPI.transfer(address);
  delayMicroseconds(50);
  data = SPI.transfer(0x00);
  //pinMode(MOSI, OUTPUT);
  // take the SS pin high to de-select the chip:
  digitalWrite(ADNS3080_CHIP_SELECT,HIGH);
  return(data);
}
void ADNS3080::write(unsigned int address, unsigned char data)
{
  // take the CS pin low to select the chip:
  digitalWrite(ADNS3080_CHIP_SELECT,LOW);
  //  send in the address and value via SPI:
  SPI.transfer(0x80 | address);
  delayMicroseconds(50);
  SPI.transfer(data);
  // take the SS pin high to de-select the chip:
  digitalWrite(ADNS3080_CHIP_SELECT,HIGH);
}
void ADNS3080::getVelocity()
{
	temp = read(ADNS3080_MOTION);
	dx_cnt = read(ADNS3080_DELTA_X);
	dy_cnt = read(ADNS3080_DELTA_Y);
	surfaceQuality = getSurfaceQuality();

	if( surfaceQuality > ADNS3080_SURFACE_QUALITY_MIN)
	{
		velocity[0] = (float)dx_cnt/(ADNS3080_RESOLUTION)/dT;
		velocity[1] = (float)dy_cnt/(ADNS3080_RESOLUTION)/dT;
	}
	else
	{
		velocity[0] = velocity[1] = 0;
	}

	rotateAboutZAxis(-M_PI/4,&velocity[0]); //align with rotor arm axis
//	velocity[1] = -velocity[1];
}
void ADNS3080::getConfig()
{
	char config = read(ADNS3080_CONFIGURATION_BITS);

	resolution = config | 0b10000;
	ledAlwaysOn = config | 0b1000000;
}
int ADNS3080::getResolution()
{
  if ( (read(ADNS3080_CONFIGURATION_BITS) & 0x10) == 0 )
    return 400;
  else
    return 1600;
}
void ADNS3080::setResolution(const char resolution)
{
	char config = read(ADNS3080_CONFIGURATION_BITS);
	if( resolution != ( (config & 0x10)>>4 ) )
	{
		delayMicroseconds(50);
		write(ADNS3080_CONFIGURATION_BITS,config ^ 0x10);
	}
}

char ADNS3080::getPID()
{
	char data = read(ADNS3080_PRODUCT_ID);
	return data;
}

uint8_t ADNS3080::getSurfaceQuality()
{
	uint8_t data = read(ADNS3080_SQUAL);
	return data*4;
}

