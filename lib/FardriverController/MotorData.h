/*

*/

#pragma once

#include <inttypes.h>

// 0x0100	BlockInfo
// 0x0101	BlockHealth
// 0x0102	BlockCfg
// 0x0103	BlockError
struct __attribute__((__packed__)) MotorData
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
	// 0x010B	TemperatureMotor
	int16_t TMotor;
	// 0x010C	TemperatureController
	int16_t TController;
	// 0x010D	Odometer
	uint32_t Odometer;

	uint8_t D2;
};


enum motor_gear_t
{
	MOTOR_GEAR_NEUTRAL = 0x00,
	MOTOR_GEAR_FORWARD = 0x01,
	MOTOR_GEAR_REVERSE = 0x02,
	MOTOR_GEAR_LOW = 0x03,
	MOTOR_GEAR_BOOST = 0x04,
	MOTOR_GEAR_UNKNOWN = 0x0F
};

enum motor_roll_t
{
	MOTOR_ROLL_STOP = 0x00,
	MOTOR_ROLL_FORWARD = 0x01,
	MOTOR_ROLL_REVERSE = 0x02,
	MOTOR_ROLL_UNKNOWN = 0x0F
};
