#pragma once
#include <inttypes.h>

namespace Config
{
	struct __attribute__((packed)) eeprom_body_t
	{
		uint16_t wheel_diameter = 620;		// Диаметр колеса, мм.

		struct
		{
			//uint16_t pedal_min = 650;	// Минимальное значение ADC при отпущенной педали газа
			//uint16_t pedal_max = 3500;	// Максимальное значение ADC при нажатой педали газа
			uint16_t pwm_min = 0;		// Минимальное значение PWM при отпущенной педали газа
			uint16_t pwm_max = 1024;	// Максимальное значение PWM при нажатой педали газа
		} throttle;
	};
};
