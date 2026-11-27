#pragma once
#include <OneWireDriver.h>
#include <OneWireTSens.h>
#include <CanObj/ICanStream.hpp>

extern ICanStream &CanTempStream;

namespace OneWire
{
	TIM_HandleTypeDef htim1;

	OneWireDriver bus(GPIOB, GPIO_PIN_9, &htim1);
	OneWireTSens<16> sensors(bus);

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

		sensors.RegReadyCallback([](const OneWireTSensor *sensor, uint8_t count)
		{
			for(uint8_t i = 0; i < count; ++i)
			{
				//if(sensor[i].IsValid() == false) continue;
				CanTempStream.Set(i, sensor[i].GetTemp() / 10);

				OneWireDriver::rom_t rom = sensor[i].GetRom();

				Logger.Printf("Rom: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X, Temp: % 5d°C, Valid: %d, Min: %d, Mid: %d, Max: %d", 
				rom.raw[0], rom.raw[1], rom.raw[2], rom.raw[3], 
				rom.raw[4], rom.raw[5], rom.raw[6], rom.raw[7], 
				sensor[i].GetTemp(), sensor[i].IsValid(), sensors.GetMinTemp(), sensors.GetAvgTemp(), sensors.GetMaxTemp()).PrintNewLine();
			}
			DEBUG_LOG_SIMPLE("----\n");
		});	
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		sensors.Processing(current_time);
		
		current_time = HAL_GetTick();
		return;
	}
};
