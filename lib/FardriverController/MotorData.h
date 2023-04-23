/*

*/

#pragma once

#include <inttypes.h>

// 0x0100	BlockInfo
// 0x0101	BlockHealth
// 0x0102	BlockCfg
// 0x0103	BlockError
struct __attribute__((__packed__))  MotorData
{
	// 0x0104	ControllerErrors
	uint16_t Errors;
	// 0x0105	RPM
	uint16_t RPM;
	// 0x0106	Speed
	uint16_t Speed;
	// 0x0107	Voltage
	uint16_t Voltage;
	// 0x0108	Current
	int16_t Current;
	// 0x0109	Power
	int16_t Power;
	// 0x010A	Gear+Roll
	uint8_t Gear;
	uint8_t Roll;
	// 0x010B	Temperature
	int8_t TMotor;
	int8_t TController;

	uint8_t D2;
};
