#include "floki.h"

bool sendState = false;
Metro loopTimer = Metro(100);

void setup()
{
	WIXEL::initialise();
	ENCODER::initialise();
	PWM::initialise();
	MOTORSPEEDCONTROL::initialise();
	RATECONTROL::initialise();
	LED::initialise();
	IMU::initialise();
	ECF::initialise();

	NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
	delay(100);
	IMU::getInitialConditions();
	//WATCHDOG::initialise();
	loopTimer.interval(dT*1.0e3); //msec
	analogReadRes(16);
}
void loop()
{
	WIXEL::getData();
	if( loopTimer.check() )
	{
		float temp = (float)analogRead(15);
		WIXEL::aRead = LPF2(temp, dT, 2*M_PI*5, &WIXEL::aRead_buffer[0], &WIXEL::aRead_buffer[2]);
//		WIXEL::aRead = temp;
		IMU::getReadings();
		ECF::iterate();
		THRUSTCONTROL::iterate();
		RATECONTROL::iterate();
		PWM::servoMapping();
		MOTORSPEEDCONTROL::iterate();
		PWM::write();
		LED::controlIR();

		if( sendState )
		{
			WIXEL::sendData();
			ENCODER::watchDog();
		}
		sendState = !sendState;
	}
	WATCHDOG::refresh();
}
