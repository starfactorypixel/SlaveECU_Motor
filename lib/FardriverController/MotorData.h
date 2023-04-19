/*

*/

#pragma once

#include <inttypes.h>

struct MotorData
{
	uint16_t RPM;
	uint16_t Voltage;
	int16_t Current;
	int16_t Power;
	uint8_t Gear;
	uint8_t Roll;
	uint8_t TMotor;
	uint8_t TController;
	uint32_t Odometer;
	uint8_t D2;
	uint16_t Errors;
};
