#pragma once

#include <LEDLibrary.h>

namespace Leds
{
	InfoLeds<4> ledsObj;

	inline void Setup()
	{
		ledsObj.AddLed( {GPIOC, GPIO_PIN_14}, ledsObj.LED_RED );
		ledsObj.AddLed( {GPIOC, GPIO_PIN_15}, ledsObj.LED_YELLOW );
		ledsObj.AddLed( {GPIOA, GPIO_PIN_0}, ledsObj.LED_GREEN );
		ledsObj.AddLed( {GPIOA, GPIO_PIN_1}, ledsObj.LED_BLUE );

		ledsObj.SetOn(ledsObj.LED_RED);
		HAL_Delay(100);
		ledsObj.SetOff(ledsObj.LED_RED);

		ledsObj.SetOn(ledsObj.LED_YELLOW);
		HAL_Delay(100);
		ledsObj.SetOff(ledsObj.LED_YELLOW);

		ledsObj.SetOn(ledsObj.LED_GREEN);
		HAL_Delay(100);
		ledsObj.SetOff(ledsObj.LED_GREEN);

		ledsObj.SetOn(ledsObj.LED_BLUE);
		HAL_Delay(100);
		ledsObj.SetOff(ledsObj.LED_BLUE);
	}

	inline void Loop(uint32_t &current_time)
	{
		ledsObj.Processing(current_time);

		current_time = HAL_GetTick();
	}
}
