#pragma once
#include <inttypes.h>
#include <CUtils.h>

namespace TempStream
{
	void OnTemperatureStream(auto coll_el, uint8_t idx);
	
	static constexpr uint8_t TEMP_COUNT = 16;
	
	int8_t temperatures[TEMP_COUNT];
	CollectionStream<int8_t> TemperatureStream(temperatures, sizeofarray(temperatures), OnTemperatureStream);
	
	
	void OnTemperatureStream(auto coll_el, uint8_t idx)
	{
		CANLib::obj_temperature_ext.SetValue(0, (idx + 1), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
		CANLib::obj_temperature_ext.SetValue(1, coll_el, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
		
		return;
	}
	
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
		CANLib::obj_temperature_ext.RegisterFunctionRequest([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			TemperatureStream.Start(100);
			
			return CAN_RESULT_IGNORE;
		});
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		TemperatureStream.Processing(current_time);
		
		
		// При выходе обновляем время
		current_time = HAL_GetTick();
		
		return;
	}
};
