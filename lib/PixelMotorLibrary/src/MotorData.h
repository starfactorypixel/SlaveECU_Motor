#pragma once
#include <inttypes.h>

namespace MotorManagerData
{
	// 
	enum gear_t : uint8_t
	{
		GEAR_NEUTRAL = 0x00,
		GEAR_FORWARD = (1 << 0),
		GEAR_REVERSE = (1 << 1),
		GEAR_LOW = (1 << 2),
		GEAR_BOOST = (1 << 3),
		GEAR_UNKNOWN = 0xFF
	};
	
	// 
	enum roll_t : uint8_t
	{
		ROLL_STOP = 0x00,
		ROLL_FORWARD = (1 << 0),
		ROLL_REVERSE = (1 << 1),
		ROLL_UNKNOWN = 0xFF
	};

	struct common_data_t
	{
		uint16_t errors;
		uint16_t rpm;				// 1.0 RPM
		gear_t gear;				// by gear_t
		roll_t roll;				// by roll_t
		uint16_t voltage;			// 0.1 V
		int16_t current;			// 0.1 A
		int16_t throttle;			// 1.0 %
		int16_t temp_controller;	// 1.0 C
		int16_t temp_motor;			// 1.0 C
		uint32_t odometer;			// 100 m
		uint16_t speed;				// 0.1 kmh
		int16_t power;				// 1.0 W
		uint16_t throttle_voltage;	// 10 mV
	};
}
