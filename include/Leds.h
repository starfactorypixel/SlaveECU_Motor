#pragma once

#include <LEDLibrary.h>

namespace Leds
{
	static constexpr uint8_t CFG_LedCount = 4;
	
	enum leds_t : uint8_t
	{
		LED_NONE = 0,
		LED_RED = 1,
		LED_YELLOW = 2,
		LED_GREEN = 3,
		LED_BLUE = 4,
	};
	
	InfoLeds<CFG_LedCount> obj;
	
	inline void Setup()
	{
		obj.AddLed( {GPIOC, GPIO_PIN_14}, LED_RED );
		obj.AddLed( {GPIOC, GPIO_PIN_15}, LED_YELLOW );
		obj.AddLed( {GPIOA, GPIO_PIN_0}, LED_GREEN );
		obj.AddLed( {GPIOA, GPIO_PIN_1}, LED_BLUE );

		obj.SetOn(LED_RED);
		HAL_Delay(100);
		obj.SetOff(LED_RED);

		obj.SetOn(LED_YELLOW);
		HAL_Delay(100);
		obj.SetOff(LED_YELLOW);

		obj.SetOn(LED_GREEN);
		HAL_Delay(100);
		obj.SetOff(LED_GREEN);

		obj.SetOn(LED_BLUE);
		HAL_Delay(100);
		obj.SetOff(LED_BLUE);
	}

	inline void Loop(uint32_t &current_time)
	{
		obj.Processing(current_time);

		current_time = HAL_GetTick();
	}
}
