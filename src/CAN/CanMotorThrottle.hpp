#pragma once
#include <inttypes.h>
#include <CanObjectBase.h>

class CanMotorThrottle : public CANObjectBase
{
	struct __attribute__((packed)) set_real_time_t { uint8_t fId; uint8_t counter; uint16_t val; };
	
	using function_ctrl_t = void (*)(uint8_t idx, uint16_t value);
	
	public:
		CanMotorThrottle(can_object_id_t id, uint8_t idx, function_ctrl_t ctrl) : CANObjectBase(id), _idx(idx), _ctrl(ctrl)
		{
			return;
		}
		
	protected:
		void handlerSetReadTimeFunction(set_real_time_t *obj)
		{
			uint32_t time = GetParent()->GetTime();
			
			if(_connected == false)
			{
				_connected = true;
				_last_counter = obj->counter;
				_last_time = time;
				
				_ctrl(_idx, obj->val);
				return;
			}
			
			int8_t diff = (int8_t)(obj->counter - _last_counter);
			bool counter_ok = (diff != 0) && (diff >= -2) && (diff <= 2);
			bool time_ok = (time - _last_time) <= 300;
			if(counter_ok && time_ok)
			{
				_last_counter = obj->counter;
				_last_time = time;
				
				_ctrl(_idx, obj->val);
			}
			else
			{
				// Потеряли синхронизацию
				_connected = false;
			}
			
			return;
		}
		
		virtual void OnTick(uint32_t time) noexcept override
		{
			if(_connected && (time - _last_time > 300))
			{
				// Таймаут - пакеты перестали приходить
				_connected = false;
			}
			
			return;
		}
		
		virtual void OnProcessFrame(can_frame_t &can_frame) noexcept override
		{
			uint8_t fId = can_frame.raw_data[0];
			switch(fId)
			{
				case CAN_FUNC_SET_REAL_TIME_IN:
				{
					set_real_time_t *obj = (set_real_time_t *)can_frame.raw_data;
					handlerSetReadTimeFunction(obj);
					break;
				}
			}

			return;
		}
		
	private:
		uint8_t _idx;
		function_ctrl_t _ctrl;

		bool _connected = false;
		uint8_t _last_counter;
		uint32_t _last_time;
		
};
