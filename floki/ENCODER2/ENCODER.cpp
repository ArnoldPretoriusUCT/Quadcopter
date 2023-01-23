/*
 * ENCODER.cpp
 *
 *  Created on: 03 Dec 2015
 *      Author: Arnold
 */
#include "ENCODER.h"

uint32_t ENCODER::FTM1_Ch0Count,
	     ENCODER::FTM1_Ch1Count,
	     ENCODER::FTM2_Ch0Count,
	     ENCODER::FTM2_Ch1Count;
uint16_t ENCODER::FTM1_Ch0Count_past,
	     ENCODER::FTM1_Ch1Count_past,
	     ENCODER::FTM2_Ch0Count_past,
	     ENCODER::FTM2_Ch1Count_past;
uint32_t ENCODER::FTM1_tickCount[2],
		 ENCODER::FTM2_tickCount[2];
uint8_t ENCODER::FTM1_overflowCount[2],
		ENCODER::FTM2_overflowCount[2],
		ENCODER::watchDogCount;
bool ENCODER::FTM1_isValid[2],
	 ENCODER::FTM2_isValid[2];
float ENCODER::motorSpeed[4];

void ENCODER::initialise()
{
	//FTM1
	FTM1_Ch0Count_past = 0;
	FTM1_Ch1Count_past = 0;

	// Input filter to help prevent glitches from triggering the capture
	// 4+4×val clock cycles, 48MHz = 4+4*7 = 32 clock cycles = 0.75us
	FTM1_FILTER = 0xFF;
	// Must set the Write-protect disable (WPDIS) bit to allow modifying other registers
	// The enable (FTMEN) bit is also set to enable the FlexTimer0 module
	// FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=1
	FTM1_MODE = 0x05;
	//FTM1_EXTTRIG = 0x00;
	// FLEXTimer0 configuration
	FTM1_SC = 0x00; // Set this to zero before changing the modulus
	FTM1_CNT = 0x00; // Reset the count to zero
	FTM1_MOD = 0xFFFF; // max modulus = 65535
	FTM1_SC = 0x4F;//0xF; // TOF=0 TOIE=1 CPWMS=0 CLKS=01 (sys clock) PS=111 (divide by 128)
	FTM1_C0SC = 0x48; // CHF=0 CHIE=1 (enable interrupt) MSB=0 MSA=0 ELSB=1 (input capture) ELSA=0 DMA=0
	FTM1_C1SC = 0x48;

	// Enable FTM1 interrupt inside NVIC
	NVIC_ENABLE_IRQ(IRQ_FTM1);

	// Alt function
	PORTA_PCR12 |= (0b11 << 8); //FTM1_CH0
	PORTA_PCR13 |= (0b11 << 8); //FTM1_CH1

	FTM1_isValid[0] = FTM1_isValid[1] = false;

	//FTM2
	FTM2_Ch0Count_past = 0;
	FTM2_Ch1Count_past = 0;

	// Input filter to help prevent glitches from triggering the capture
	// 4+4×val clock cycles, 48MHz = 4+4*7 = 32 clock cycles = 0.75us
	FTM2_FILTER = 0xFF;
	// Must set the Write-protect disable (WPDIS) bit to allow modifying other registers
	// The enable (FTMEN) bit is also set to enable the FlexTimer0 module
	// FAULTIE=0, FAULTM=00, CAPTEST=0, PWMSYNC=0, WPDIS=1, INIT=0, FTMEN=1
	FTM2_MODE = 0x05;
	//FTM1_EXTTRIG = 0x00;
	// FLEXTimer0 configuration
	FTM2_SC = 0x00; // Set this to zero before changing the modulus
	FTM2_CNT = 0x00; // Reset the count to zero
	FTM2_MOD = 0xFFFF; // max modulus = 65535
	FTM2_SC = 0x4F;//0xF; // TOF=0 TOIE=1 CPWMS=0 CLKS=01 (sys clock) PS=111 (divide by 128)
	FTM2_C0SC = 0x48; // CHF=0 CHIE=1 (enable interrupt) MSB=0 MSA=0 ELSB=1 (input capture) ELSA=0 DMA=0
	FTM2_C1SC = 0x48;

	// Enable FTM2 interrupt inside NVIC
	NVIC_ENABLE_IRQ(IRQ_FTM2);

	// Alt function
	PORTB_PCR18 |= (0b11 << 8); //FTM2_CH0
	PORTB_PCR19 |= (0b11 << 8); //FTM2_CH1

	FTM2_isValid[0] = FTM2_isValid[1] = false;
}

extern "C" void ftm1_isr(void)
{
	if( ENCODER::FTM1_overflowCount[0] >= 2 )
	{
		ENCODER::FTM1_isValid[0] = false;
		ENCODER::FTM1_overflowCount[0] = 2;
	}
	else
	{
		ENCODER::FTM1_isValid[0] = true;
	}
	if( ENCODER::FTM1_overflowCount[1] >= 2 )
	{
		ENCODER::FTM1_isValid[1] = false;
		ENCODER::FTM1_overflowCount[1] = 2;
	}
	else
	{
		ENCODER::FTM1_isValid[1] = true;
	}
	if ( (FTM1_SC & FTM_SC_TOF) != 0 )
	{
		ENCODER::FTM1_overflowCount[0]++;
		ENCODER::FTM1_overflowCount[1]++;

		FTM1_CNT = 0x00;
		FTM1_SC &= ~FTM_SC_TOF;
	}
	if( (FTM1_C0SC & FTM_SC_TOF) != 0 )
	{
		if( ENCODER::FTM1_isValid[0] == true )
		{
			if( ENCODER::FTM1_overflowCount[0] == 1 )
			{
				ENCODER::FTM1_Ch0Count = (FTM1_C0V+65535) - ENCODER::FTM1_Ch0Count_past;
			}
			else
			{
				ENCODER::FTM1_Ch0Count = FTM1_C0V - ENCODER::FTM1_Ch0Count_past;
			}
		}
		ENCODER::FTM1_Ch0Count_past = FTM1_C0V;
		ENCODER::FTM1_overflowCount[0] = 0;
		FTM1_C0SC &= ~0x80;
 	}
	if( (FTM1_C1SC & FTM_SC_TOF) != 0 )
  	{
		if( ENCODER::FTM1_isValid[1] == true )
		{
			if( ENCODER::FTM1_overflowCount[1] == 1 )
			{
				ENCODER::FTM1_Ch1Count = (FTM1_C1V+65535) - ENCODER::FTM1_Ch1Count_past;
			}
			else
			{
				ENCODER::FTM1_Ch1Count = FTM1_C1V - ENCODER::FTM1_Ch1Count_past;
			}
		}
		ENCODER::FTM1_Ch1Count_past = FTM1_C1V;
		ENCODER::FTM1_overflowCount[1] = 0;
		FTM1_C1SC &= ~0x80;
  	}
}

extern "C" void ftm2_isr(void)
{
	if( ENCODER::FTM2_overflowCount[0] >= 2 )
	{
		ENCODER::FTM2_isValid[0] = false;
		ENCODER::FTM2_overflowCount[0] = 2;
	}
	else
	{
		ENCODER::FTM2_isValid[0] = true;
	}
	if( ENCODER::FTM2_overflowCount[1] >= 2 )
	{
		ENCODER::FTM2_isValid[1] = false;
		ENCODER::FTM2_overflowCount[1] = 2;
	}
	else
	{
		ENCODER::FTM2_isValid[1] = true;
	}
	if ( (FTM2_SC & FTM_SC_TOF) != 0 )
	{
		ENCODER::FTM2_overflowCount[0]++;
		ENCODER::FTM2_overflowCount[1]++;

		FTM2_CNT = 0x00;
		FTM2_SC &= ~FTM_SC_TOF;
	}
	if( (FTM2_C0SC & FTM_SC_TOF) != 0 )
	{
		if( ENCODER::FTM2_isValid[0] == true )
		{
			if( ENCODER::FTM2_overflowCount[0] == 1 )
			{
				ENCODER::FTM2_Ch0Count = (FTM2_C0V+65535) - ENCODER::FTM2_Ch0Count_past;
			}
			else
			{
				ENCODER::FTM2_Ch0Count = FTM2_C0V - ENCODER::FTM2_Ch0Count_past;
			}
		}
		ENCODER::FTM2_Ch0Count_past = FTM2_C0V;
		ENCODER::FTM2_overflowCount[0] = 0;
		FTM2_C0SC &= ~0x80;
 	}
	if( (FTM2_C1SC & FTM_SC_TOF) != 0 )
  	{
		if( ENCODER::FTM2_isValid[1] == true )
		{
			if( ENCODER::FTM2_overflowCount[1] == 1 )
			{
				ENCODER::FTM2_Ch1Count = (FTM2_C1V+65535) - ENCODER::FTM2_Ch1Count_past;
			}
			else
			{
				ENCODER::FTM2_Ch1Count = FTM2_C1V - ENCODER::FTM2_Ch1Count_past;
			}
		}
		ENCODER::FTM2_Ch1Count_past = FTM2_C1V;
		ENCODER::FTM2_overflowCount[1] = 0;
		FTM2_C1SC &= ~0x80;
  	}
}

void ENCODER::getMotorSpeed()
{
	//FTM1
	if( FTM1_isValid[0] == false )
	{
		motorSpeed[3] = 0;
	}
	else
	{
		motorSpeed[3] = ENCODER_CPU_FRQ / ENCODER_PSC / (float)FTM1_Ch0Count * 2 * M_PI;
	}
	if( FTM1_isValid[1] == false )
	{
		motorSpeed[2] = 0;
	}
	else
	{
		motorSpeed[2] = ENCODER_CPU_FRQ / ENCODER_PSC / (float)FTM1_Ch1Count * 2 * M_PI;
	}
	//FTM2
	if( FTM2_isValid[0] == false )
	{
		motorSpeed[0] = 0;
	}
	else
	{
		motorSpeed[0] = ENCODER_CPU_FRQ / ENCODER_PSC / (float)FTM2_Ch0Count * 2 * M_PI;
	}
	if( FTM2_isValid[1] == false )
	{
		motorSpeed[1] = 0;
	}
	else
	{
		motorSpeed[1] = ENCODER_CPU_FRQ / ENCODER_PSC / (float)FTM2_Ch1Count * 2 * M_PI;
	}
}

void ENCODER::watchDog()
{
//	if( !PWM::motorsArmed || PWM::isCalibrating ){ return; }
	if(1){ return; }
	if( (PWM::escCom[0] > ENCODER_ESC_THRESHOLD) || (PWM::escCom[1] > ENCODER_ESC_THRESHOLD) )
	{
		if( !FTM1_isValid[0] || !FTM1_isValid[1] || !FTM2_isValid[0] || !FTM2_isValid[1] )
		{
			watchDogCount++;
		}
		else
		{
			watchDogCount = 0;
		}
		if( watchDogCount >= ENCODER_WATCHDOG_THRESHOLD )
		{
			WATCHDOG::isCritical = true;
			digitalWrite(LED_FLAG2_PIN,true);
		}
	}
	if( MOTORSPEEDCONTROL::isClosedLoop )
	{

	}
}
