/*
 * WIXEL.cpp
 *
 *  Created on: 06 Jun 2016
 *      Author: Arnold
 */

#include "WIXEL.h"

uint16_t WIXEL::packetCount;
float WIXEL::receiveFrequency, WIXEL::aRead, WIXEL::aRead_buffer[4];
bool WIXEL::isOpen,WIXEL::commsState,WIXEL::newData;
uint8_t WIXEL::packetID;

char WIXEL::watchDogCount;
unsigned char WIXEL::receiveBuffer[WIXEL_RECEIVE_BUFFER_SIZE],
		  	  WIXEL::sendBuffer[WIXEL_SEND_BUFFER_SIZE];
float WIXEL::sendTime[2],
	  WIXEL::receiveTime[2];

void WIXEL::initialise()
{
	packetCount = 0;
	watchDogCount = 0;
	isOpen = false;
	commsState = true;
	newData = false;

	WIXEL_TX.begin(WIXEL_BAUD_RATE);//
	WIXEL_TX.clear();
	WIXEL_RX.begin(WIXEL_BAUD_RATE);
	WIXEL_RX.clear();

	packetID = 0;
}

void WIXEL::getData()
{
	if( WIXEL_RX.available() >= WIXEL_RECEIVE_BUFFER_SIZE )
	{
		WIXEL_RX.readBytes(&receiveBuffer[0], WIXEL_RECEIVE_BUFFER_SIZE);
		WIXEL_RX.clear();

		static bool radioIsOpen_temp = false;
		radioIsOpen_temp = RADIO::isOpen;
		if( (receiveBuffer[0] == '*') && (receiveBuffer[WIXEL_RECEIVE_BUFFER_SIZE-1] == '!') )
		{
			watchDogCount = 0;
			isOpen = true;
			newData = true;

			unsigned char index = 1;
			//heave command
			HEAVECONTROL::u = uchars2float(&receiveBuffer[index],WIXEL_HEAVE_SF); 			index += 2;
			//reference quaternion vector
			ATTITUDECONTROL::qReference[1] = chars2float(&receiveBuffer[index],WIXEL_QUATERNION_SF); index += 2;
			ATTITUDECONTROL::qReference[2] = chars2float(&receiveBuffer[index],WIXEL_QUATERNION_SF); index += 2;
			ATTITUDECONTROL::qReference[3] = chars2float(&receiveBuffer[index],WIXEL_QUATERNION_SF); index += 2;
			//camera quaternion vector
			ECF::qCamera[1] = chars2float(&receiveBuffer[index],WIXEL_QUATERNION_SF); index += 2;
			ECF::qCamera[2] = chars2float(&receiveBuffer[index],WIXEL_QUATERNION_SF); index += 2;
			ECF::qCamera[3] = chars2float(&receiveBuffer[index],WIXEL_QUATERNION_SF); index += 2;
			//motor speed ref
			MOTORSPEEDCONTROL::reference = (float)receiveBuffer[index]*800/255; 			index++;
			//state vector
			unsigned char stateVector = receiveBuffer[index];								index++;
			RADIO::isOpen						 = (stateVector >> 7) & 0b1;
			ATTITUDECONTROL::isClosedLoop        = (stateVector >> 6) & 0b1;
			LED::isEnabled                       = (stateVector >> 5) & 0b1;
			WATCHDOG::resetMicro  				 = (stateVector >> 4) & 0b1;
			PWM::isCalibrating                   = (stateVector >> 3) & 0b1;
			PWM::motorsArmed                     = (stateVector >> 2) & 0b1;
			MOTORSPEEDCONTROL::torqueFeedForward = (stateVector >> 1) & 0b1;
			MOTORSPEEDCONTROL::isClosedLoop      = stateVector & 0b1;

			getReceiveFrequency();
		}
		if( radioIsOpen_temp && !RADIO::isOpen )
		{
			WATCHDOG::isCritical = true;
//			Serial.println("RADIO:criticalFailure");
		}
//		sendData();
	}
}

void WIXEL::sendData()
{
	sendTime[1] = sendTime[0];
	sendTime[0] = micros();
	commsState = false;

	sendBuffer[0]  = '*';
	sendBuffer[WIXEL_SEND_BUFFER_SIZE-1] = '!';
	unsigned char index = 1;
//	float2chars(IMU::gyro[0],WIXEL_GYRO_SF, &sendBuffer[index]);							index += 2;
	float2chars(aRead,1, &sendBuffer[index]);												index += 2;
	float2chars(IMU::gyro[1],WIXEL_GYRO_SF, &sendBuffer[index]);							index += 2;
//	float2chars(MOTORSPEEDCONTROL::current[2],40,&sendBuffer[index]);						index += 2;
	float2chars(IMU::gyro[2],WIXEL_GYRO_SF, &sendBuffer[index]);							index += 2;
//	float2chars(PWM::escCom[2],1, &sendBuffer[index]);										index += 2;
	float2chars(ECF::qFused[1],WIXEL_QUATERNION_SF, &sendBuffer[index]);					index += 2;
	float2chars(ECF::qFused[2],WIXEL_QUATERNION_SF, &sendBuffer[index]);					index += 2;
	float2chars(ECF::qFused[3],WIXEL_QUATERNION_SF, &sendBuffer[index]);					index += 2;
	sendBuffer[index] = float2UINT8(THRUSTCONTROL::current[0],WIXEL_CURRENT_SF);		index++;
	sendBuffer[index] = float2UINT8(THRUSTCONTROL::current[1],WIXEL_CURRENT_SF);		index++;
	sendBuffer[index] = float2UINT8(THRUSTCONTROL::current[2],WIXEL_CURRENT_SF);		index++;
	sendBuffer[index] = float2UINT8(THRUSTCONTROL::current[3],WIXEL_CURRENT_SF);		index++;
	sendBuffer[index] = float2INT8(-PWM::servoComDelta[0],WIXEL_SERVO_SF);					index++;
	sendBuffer[index] = float2INT8(PWM::servoComDelta[1],WIXEL_SERVO_SF);					index++;
	sendBuffer[index] = float2INT8(-PWM::servoComDelta[2],WIXEL_SERVO_SF);					index++;
	sendBuffer[index] = float2INT8(PWM::servoComDelta[3],WIXEL_SERVO_SF);					index++;
	sendBuffer[index] = float2UINT8(ENCODER::motorSpeed[0],WIXEL_MOTOR_SF); 				index++;
	sendBuffer[index] = float2UINT8(ENCODER::motorSpeed[1],WIXEL_MOTOR_SF); 				index++;
	sendBuffer[index] = float2UINT8(ENCODER::motorSpeed[2],WIXEL_MOTOR_SF); 				index++;
	sendBuffer[index] = float2UINT8(ENCODER::motorSpeed[3],WIXEL_MOTOR_SF); 				index++;
	sendBuffer[index] = float2UINT8(MOTORSPEEDCONTROL::batteryVoltage,WIXEL_V_BAT_SF);		index++;

	WIXEL_TX.clear();
	WIXEL_TX.write(&sendBuffer[0],WIXEL_SEND_BUFFER_SIZE);

	static uint16_t startCnt = 0;
	if(startCnt > 700)
	{
		WIXEL::watchDog();
		startCnt = 700;
	}
	startCnt++;
}

void WIXEL::getReceiveFrequency()
{
	receiveTime[1] = receiveTime[0];
	receiveTime[0] = micros();
	uint16_t dt = receiveTime[0]-receiveTime[1];
	if( dt != 0 )
	{
		receiveFrequency = 1/(float)dt;
	}
	else receiveFrequency = 0;
}

void WIXEL::watchDog()
{
	if( watchDogCount >= WIXEL_RECEIVE_WATCH_DOG_THRESHOLD )
	{
		receiveFrequency = 0;
		isOpen = false;
		WATCHDOG::isCritical = true;
		digitalWrite(LED_FLAG1_PIN,true);
		watchDogCount = WIXEL_RECEIVE_WATCH_DOG_THRESHOLD;
	}
	watchDogCount++;
}
