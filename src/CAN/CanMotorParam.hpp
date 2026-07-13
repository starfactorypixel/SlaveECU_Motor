#pragma once
#include <inttypes.h>
#include <CanObjectBase.h>

template <typename T> 
class CanMotorParam : public CANObjectBase
{
	//using T = uint16_t;
	struct __attribute__((packed)) request_t { uint8_t fId; };
	struct __attribute__((packed)) timer_t { uint8_t fId; T val; };
	struct __attribute__((packed)) event_ok_t { uint8_t fId = CAN_FUNC_EVENT_OK; T val; };
	
	using value_classifier_t = uint8_t (*)(T value);
	
	public:
		CanMotorParam(can_object_id_t id, T *value_ptr, uint16_t timer) : CANObjectBase(id), _value_ptr(value_ptr)
		{
			this->SetTimerPeriod(timer);

			return;
		}

		void SetValueClassifier(value_classifier_t function)
		{
			_value_classifier = function;
			
			return;
		}
		
	protected:
		void handlerRequestFunction(request_t *obj)
		{
			event_ok_t answer = {};
			answer.val = *_value_ptr;
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
			T val = *_value_ptr;
			
			timer_t answer = {};
			answer.fId = (_value_classifier == nullptr) ? CAN_FUNC_TIMER_NORMAL : _value_classifier(val);
			answer.val = val;
			this->SendFrame((uint8_t *)&answer, sizeof(answer));
		}
		
	private:
		T *_value_ptr;
		value_classifier_t _value_classifier = nullptr;
};
