#pragma once
#include <inttypes.h>
#include <CanObjectBase.h>

class CanMotorGearRollParam : public CANObjectBase
{
	struct __attribute__((packed)) request_t { uint8_t fId; };
	struct __attribute__((packed)) timer_t { uint8_t fId; uint8_t gear; uint8_t roll; };
	struct __attribute__((packed)) event_ok_t { uint8_t fId = CAN_FUNC_EVENT_OK; uint8_t gear; uint8_t roll; };
	
	public:
		CanMotorGearRollParam(can_object_id_t id, MotorManagerData::gear_t *gear_ptr, MotorManagerData::roll_t *roll_ptr, uint16_t timer) : CANObjectBase(id), _gear_ptr(gear_ptr), _roll_ptr(roll_ptr)
		{
			this->SetTimerPeriod(timer);

			return;
		}
		
	protected:
		void handlerRequestFunction(request_t *obj)
		{
			event_ok_t answer = {};
			answer.gear = *_gear_ptr;
			answer.roll = *_roll_ptr;
			this->SendFrame((uint8_t *)&answer, sizeof(answer));
			
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
				case CAN_FUNC_REQUEST_IN:
				{
					request_t *obj = (request_t *)can_frame.raw_data;
					handlerRequestFunction(obj);
					break;
				}
			}

			return;
		}

		virtual void OnTimer() noexcept override
		{
			timer_t answer = {};
			answer.fId = CAN_FUNC_TIMER_NORMAL;
			answer.gear = *_gear_ptr;
			answer.roll = *_roll_ptr;
			this->SendFrame((uint8_t *)&answer, sizeof(answer));
		}
		
	private:
		MotorManagerData::gear_t *_gear_ptr;
		MotorManagerData::roll_t *_roll_ptr;
};
