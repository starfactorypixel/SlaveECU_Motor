#pragma once
#include <inttypes.h>
#include <CUtils_Analog.h>

extern ADC_HandleTypeDef hadc1;

namespace Analog
{
	
	// Входные АЦП порты, обрабатываемые регулярной группой
	enum port_regular_t : uint8_t
	{
		PORT_REG_NONE,
		PORT_REG_STMTEMP
	};
	
	struct regular_channel_t
	{
		GPIO_TypeDef *port;
		uint32_t pin;
		uint32_t channel;
		uint32_t rank;
	};
	
	
	static constexpr regular_channel_t channels[] = 
	{
		{GPIOA, 0, ADC_CHANNEL_TEMPSENSOR, ADC_REGULAR_RANK_1}
	};
	static constexpr uint8_t regular_channel_count = sizeofarray(channels);
	volatile uint16_t regular_buf[regular_channel_count];
	
	
	const uint16_t GetRegularValue(/*port_regular_t*/ uint8_t num)
	{
		if(--num >= regular_channel_count) return 0;

		return regular_buf[num];
	}
	
	static void RegularConfig()
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		
		ADC_ChannelConfTypeDef sConfig = {0};
		sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

		for(auto &channel : channels)
		{
			GPIO_InitStruct.Pin = channel.pin;
			HAL_GPIO_Init(channel.port, &GPIO_InitStruct);

			sConfig.Channel = channel.channel;
			sConfig.Rank = channel.rank;
			if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			{
				Error_Handler();
			}
		}
	}
	
	inline void RegularSetup()
	{
		HAL_ADCEx_Calibration_Start(&hadc1);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)regular_buf, regular_channel_count);
		
		return;
	}
};
