#pragma once
#include <inttypes.h>
#include <CanObjectBase.h>

class CanMotorCtrl : public CANObjectBase
{
	struct __attribute__((packed)) request_t { uint8_t fId; };
	struct __attribute__((packed)) set_t { uint8_t fId; uint8_t val; };
	struct __attribute__((packed)) event_ok_t { uint8_t fId = CAN_FUNC_EVENT_OK; uint8_t val; };
	
	using function_ctrl_t = void (*)(uint8_t idx, uint8_t value);
	
	public:
		CanMotorCtrl(can_object_id_t id, uint8_t idx, function_ctrl_t ctrl) : CANObjectBase(id), _idx(idx), _ctrl(ctrl)
		{
			return;
		}
		
	protected:
		void handlerSetFunction(set_t *obj)
		{
			_ctrl(_idx, obj->val);
			
			event_ok_t answer = {};
			answer.val = obj->val;
			this->SendFrame((uint8_t *)&answer, sizeof(answer));

			return;
		}

		void handlerRequestFunction(request_t *obj)
		{
			//event_ok_t answer = {};
			//answer.val = *_value_ptr;
			//this->SendFrame((uint8_t *)&answer, sizeof(answer));
			
			return;
		}
		
		virtual void OnTick(uint32_t time) noexcept override
		{
			return;
		}

		virtual void OnProcessFrame(can_frame_t &can_frame) noexcept override
		{
			uint8_t fId = can_frame.raw_data[0];
			switch(fId)
			{
				case CAN_FUNC_SET_IN:
				{
					set_t *obj = (set_t *)can_frame.raw_data;
					handlerSetFunction(obj);
					break;
				}
				case CAN_FUNC_REQUEST_IN:
				{
					request_t *obj = (request_t *)can_frame.raw_data;
					handlerRequestFunction(obj);
					break;
				}
			}

			return;
		}
		
	private:
		uint8_t _idx;
		function_ctrl_t _ctrl;
};
