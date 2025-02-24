#pragma once
#include <OneWireTSensEx.h>

namespace OneWire
{
	TIM_HandleTypeDef htim1;
	
	OneWireDriver oneWire(GPIOB, GPIO_PIN_9, &htim1);
	OneWireTSensEx<TempStream::TEMP_COUNT> sensors(oneWire);
	
	
	static HAL_StatusTypeDef MX_TIM1_Init(void)
	{
		__HAL_RCC_TIM1_CLK_ENABLE();
		
		htim1.Instance = TIM1;
		htim1.Init.Prescaler = 63;
		htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim1.Init.Period = 65535;
		htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim1.Init.RepetitionCounter = 0;
		htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if(HAL_TIM_Base_Init(&htim1) != HAL_OK)
		{
			Error_Handler();
		}
		
		return HAL_TIM_Base_Start(&htim1);
	}
	
	
	inline void Setup()
	{
		MX_TIM1_Init();

		sensors.RegReadyCallback([](OneWireTSensEx<16>::sensor_t *obj, uint8_t count) -> void
		{
			for(uint8_t i = 0; i < count; ++i)
			{
				TempStream::PutFrom1WireByIdx(obj[i].temp, i);

				Logger.Printf("Rom: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X, Temp: %05d°C, Valid: %d, Min: %d, Mid: %d, Max: %d", 
				obj[i].rom->raw[0], obj[i].rom->raw[1], obj[i].rom->raw[2], obj[i].rom->raw[3], 
				obj[i].rom->raw[4], obj[i].rom->raw[5], obj[i].rom->raw[6], obj[i].rom->raw[7], 
				obj[i].temp, obj[i].valid, sensors.GetMinTemp(), sensors.GetMidTemp(), sensors.GetMaxTemp()).PrintNewLine();
			}
			DEBUG_LOG_STR("", "----\n");
		});		
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		sensors.Processing(current_time);
		
		
		// При выходе обновляем время
		current_time = HAL_GetTick();
		
		return;
	}
};
