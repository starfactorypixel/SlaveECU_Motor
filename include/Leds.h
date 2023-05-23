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
	
	InfoLeds<CFG_LedCount> ledsObj;
	
	inline void Setup()
	{
		ledsObj.AddLed( {GPIOC, GPIO_PIN_14}, LED_RED );
		ledsObj.AddLed( {GPIOC, GPIO_PIN_15}, LED_YELLOW );
		ledsObj.AddLed( {GPIOA, GPIO_PIN_0}, LED_GREEN );
		ledsObj.AddLed( {GPIOA, GPIO_PIN_1}, LED_BLUE );

		ledsObj.SetOn(LED_RED);
		HAL_Delay(100);
		ledsObj.SetOff(LED_RED);

		ledsObj.SetOn(LED_YELLOW);
		HAL_Delay(100);
		ledsObj.SetOff(LED_YELLOW);

		ledsObj.SetOn(LED_GREEN);
		HAL_Delay(100);
		ledsObj.SetOff(LED_GREEN);

		ledsObj.SetOn(LED_BLUE);
		HAL_Delay(100);
		ledsObj.SetOff(LED_BLUE);
	}

	inline void Loop(uint32_t &current_time)
	{
		ledsObj.Processing(current_time);

		current_time = HAL_GetTick();
	}
}
