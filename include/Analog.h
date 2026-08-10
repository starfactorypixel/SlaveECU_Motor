#pragma once
#include <AnalogRegular.h>
#include <AnalogMux.h>
#include <DrakePinA.hpp>
#include <CUtils_Analog.h>

extern ADC_HandleTypeDef hadc2;

namespace Analog
{
	uint16_t OnMuxRequest(uint8_t address);
	void OnMuxResponse(uint8_t address, uint16_t value);
	
	DrakePinA adc_pin({&hadc2, GPIOB, GPIO_PIN_1, ADC_CHANNEL_9}, ADC_SAMPLETIME_7CYCLES_5);
	DividerVoltageCalc VoltCalc(12, 3300, 69000, 10000);
	
	AnalogMux<0> mux( OnMuxRequest, OnMuxResponse
	/*,
		EasyPinD::d_pin_t{GPIOB, GPIO_PIN_4}, 
		EasyPinD::d_pin_t{GPIOB, GPIO_PIN_5}, 
		EasyPinD::d_pin_t{GPIOB, GPIO_PIN_6}, 
		EasyPinD::d_pin_t{GPIOB, GPIO_PIN_7}
	*/
	);
	
	uint16_t OnMuxRequest(uint8_t address)
	{
		return adc_pin.ReadRaw();
	}
	
	void OnMuxResponse(uint8_t address, uint16_t value)
	{
		if(address == 1)
		{

		}
		
		return;
	}
	
	uint16_t GetInsideVoltage()
	{
		const uint16_t adc = mux.Get(1);
		return VoltCalc.GetmV(adc);
	}
	
	uint16_t GetInsideCurrent()
	{
		return 0;
	}
	
	int8_t GetInsideTemperature()
	{
		const uint16_t adc = GetRegularValue(PORT_REG_STMTEMP);
		return (GetF103Temperature(adc, 3300) + 5) / 10;
	}
	
	
	inline void Setup()
	{
		RegularSetup();
		
		adc_pin.Init();
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		mux.Processing(current_time);
		
		static uint32_t tick1000 = 0;
		if(current_time - tick1000 > 1000)
		{
			tick1000 = current_time;
			
			// Раз в минуту запускаем калибровку ADC
			static uint8_t adc_calibration = 0;
			if(++adc_calibration >= 60)
			{
				adc_calibration = 0;

				adc_pin.Calibration();
			}
		}
		
		// При выходе обновляем время
		current_time = HAL_GetTick();
		
		return;
	}
};
