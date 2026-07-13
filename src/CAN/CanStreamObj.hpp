#pragma once
#include <inttypes.h>
#include <CanObjectBase.h>

template <typename T> 
class CanStreamObj : public CANObjectBase
{
	//using T = int8_t;
	struct __attribute__((packed)) request_t { uint8_t fId; };
	struct __attribute__((packed)) stream_t { uint8_t fId = 0x65; uint8_t num; T val; };
	
	public:
		CanStreamObj(can_object_id_t id, const T *collection, uint8_t length) : CANObjectBase(id), _collection(collection), _length(length)
		{

		};

		void SetInterval(uint16_t interval)
		{
			_interval = interval;
		}

	protected:
		
		void handlerRequestFunction(request_t *obj)
		{
			_last_tick = 0;
			_send_idx = 0;
			_is_run = true;
			
			return;
		}
		
		virtual void OnTick(uint32_t time) noexcept override
		{
			if(_is_run == false) return;
			
			if((time - _last_tick) < _interval) return;
			_last_tick = time;
			
			_SendStreamElement(_send_idx++);
			
			if(_send_idx >= _length)
			{
				_is_run = false;
			}

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
		
	private:
		void _SendStreamElement(uint8_t idx)
		{
			stream_t answer = {};
			answer.num = idx + 1;
			answer.val = _collection[idx];
			this->SendFrame((uint8_t *)&answer, sizeof(answer));

			return;
		}
		
		const T *_collection;
		uint8_t _length;

		uint16_t _interval = 100;
		bool _is_run = false;
		uint8_t _send_idx = 0;
		
		uint32_t _last_tick = 0;
};
