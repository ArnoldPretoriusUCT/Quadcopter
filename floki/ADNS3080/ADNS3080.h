/*
 * ADNS3080.h
 *
 *  Created on: 07 Mar 2016
 *      Author: Arnold
 */

#ifndef ADNS3080_H_
#define ADNS3080_H_

#include "Arduino.h"
#include "SPI.h"
#include "common.h"

#define ADNS3080_CHIP_SELECT 15
#define ADNS3080_RESET 21

#define ADNS3080_PRODUCT_ID            			0x00
#define ADNS3080_REVISION_ID           			0x01
#define ADNS3080_MOTION                			0x02
#define ADNS3080_DELTA_X               			0x03
#define ADNS3080_DELTA_Y              			0x04
#define ADNS3080_SQUAL                 			0x05
#define ADNS3080_PIXEL_SUM             			0x06
#define ADNS3080_MAXIMUM_PIXEL         			0x07
#define ADNS3080_CONFIGURATION_BITS    			0x0a
#define ADNS3080_EXTENDED_CONFIG       			0x0b
#define ADNS3080_DATA_OUT_LOWER        			0x0c
#define ADNS3080_DATA_OUT_UPPER        			0x0d
#define ADNS3080_SHUTTER_LOWER         			0x0e
#define ADNS3080_SHUTTER_UPPER         			0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    			0x10
#define ADNS3080_FRAME_PERIOD_UPPER    			0x11
#define ADNS3080_MOTION_CLEAR          			0x12
#define ADNS3080_FRAME_CAPTURE         			0x13
#define ADNS3080_SROM_ENABLE           			0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER	0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER   0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER   0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER   0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER        0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER        0x1e
#define ADNS3080_SROM_ID               			0x1f
#define ADNS3080_OBSERVATION           			0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    			0x3f
#define ADNS3080_PIXEL_BURST           			0x40
#define ADNS3080_MOTION_BURST          			0x50
#define ADNS3080_SROM_LOAD             			0x60

#define ADNS3080_RESOLUTION400  				0x00
#define ADNS3080_RESOLUTION1600 				0x01 //resolution seems to 800, not 1600

#define ADNS3080_RESOLUTION						(2*400/2.54e-2)
#define ADNS3080_FOCAL_LENGTH 					(1.35*4.2e-3)
#define ADNS3080_SURFACE_QUALITY_MIN 			30

class ADNS3080
{
	public:
		ADNS3080();
		void initialise();

		signed char read(unsigned char address);
		void write(unsigned int address, unsigned char data);
		void getVelocity();
		void getConfig();
		int getResolution();
		void setResolution(const char resolution);
		char getPID();
		uint8_t getSurfaceQuality();

		static uint8_t surfaceQuality;
		static signed char dx_cnt,dy_cnt;
		static float velocity[2];
		bool resolution,ledAlwaysOn;

	private:
		char temp;
};


#endif /* ADNS3080_H_ */
