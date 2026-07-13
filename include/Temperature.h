#pragma once
#include <inttypes.h>

namespace TempStream
{
	static constexpr uint8_t TEMP_COUNT = 16;
	
	int8_t temperatures[TEMP_COUNT];
	
	
	void PutFrom1Wire(int16_t *temp, uint8_t count)
	{
		if(count >= sizeofarray(temperatures))
			return;
		
		for(uint8_t i = 0; i < count; ++i)
		{
			temperatures[i] = temp[i] / 100;
		}
		
		return;
	}

	void PutFrom1WireByIdx(int16_t temp, uint8_t idx)
	{
		if(idx >= sizeofarray(temperatures))
			return;
		
		temperatures[idx] = temp / 100;
		
		return;
	}
	
	
	inline void Setup()
	{
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		
		
		current_time = HAL_GetTick();
		return;
	}
};
