#pragma once
#include <inttypes.h>
#include <string.h>
#include <CanObjectBase.h>
#include <CANBackEvents.h>

class CanBlockInfo : public CANObjectBase, public IBlockInfoSender
{
	struct __attribute__((packed)) block_wakeup_t { uint8_t fId = 0x70; uint8_t reason; };
	struct __attribute__((packed)) heartbeat_t { uint8_t fId = 0x71; uint8_t counter; };
	// uint8_t fId = 0x72; - block_info_static_t
	// uint8_t fId = 0x73; - block_info_dynamic_t
	struct __attribute__((packed)) error_t
	{
		uint8_t fId = 0x75;
		uint8_t group;				// Группа ошибки
		uint8_t code;				// Код ошибки
		uint16_t subcode;			// Доп. код ошибки
	};
	
	public:
		struct __attribute__((packed)) block_info_static_t
		{
			//uint8_t fId;				// Т.к. пакет отправляется чанками, то fId не указывается, - только данные
			uint8_t hw_ver:3;			// Версия платы, 3 бита
			uint8_t hw_type:5;			// Тип платы, 5 бит
			uint8_t can_ver:2;			// Версия протокола CAN, 2 бита
			uint8_t sw_ver:6;			// Версия программы, 6 бит
			uint8_t sn[8];				// Серийный номер блока
			uint8_t features[7];		// Возможности блока
		};
		struct __attribute__((packed)) block_info_dynamic_t
		{
			//uint8_t fId;				// Т.к. пакет отправляется чанками, то fId не указывается, - только данные
			uint32_t uptime;			// Uptime блока, мс.
			uint16_t voltage;			// Напряжение питание блока
			uint16_t current;			// Общий потребляемый ток блока
			int8_t temperature;			// Температура блока, если есть
			uint8_t error_flags;		// Флаги налчичия ошибок блока
		};
		
		using callback_static_info_request_t = void (*)(block_info_static_t &data);
		using callback_dynamic_info_request_t = void (*)(block_info_dynamic_t &data);
		
		CanBlockInfo(can_object_id_t id, callback_static_info_request_t static_callback, callback_dynamic_info_request_t dynamic_callback) : 
			CANObjectBase(id), _StaticInfoRequest(static_callback), _DynamicInfoRequest(dynamic_callback)
		{
			return;
		};
		
		virtual void SetErrorFlag(error_flag_t flag, bool state) override
		{
			if(state)
				_block_info_dynamic.error_flags |= flag;
			else
				_block_info_dynamic.error_flags &= ~flag;
			
			return;
		}
		
		virtual bool GetErrorFlag(error_flag_t flag) override
		{
			return (_block_info_dynamic.error_flags & flag) != 0;
		}
		
		virtual void SendWakeupMsg(uint8_t reason) override
		{
			_SendWakeup(reason);

			return;
		}
		
		virtual void SendErrorMsg(uint8_t group, uint8_t code, uint16_t subcode) override
		{
			_SendError(group, code, subcode);
			
			return;
		}

	protected:
		
		virtual void OnTick(uint32_t time) noexcept override
		{
			if(time - _last_heartbeat >= 5000)
			{
				_last_heartbeat = time;

				_SendHeartbeat();
			}
			
			return;
		}

		virtual void OnProcessFrame(can_frame_t &can_frame) noexcept override
		{
			uint8_t fId = can_frame.raw_data[0];
			switch(fId)
			{
				case 0x32:
				{
					_SendBlockInfoStatic();
					break;
				}
				case 0x33:
				{
					_SendBlockInfoDynamic();
					break;
				}
			}

			return;
		}

		virtual void OnTimer() noexcept override
		{
			return;
		}
		
	private:
		void _SendWakeup(uint8_t reason)
		{
			block_wakeup_t wakeup = {};
			wakeup.reason = reason;
			this->SendFrame((uint8_t *)&wakeup, sizeof(wakeup));
			
			return;
		}
		
		void _SendHeartbeat()
		{
			_heartbeat.counter += 1;
			this->SendFrame((uint8_t *)&_heartbeat, sizeof(_heartbeat));
			
			return;
		}
		
		void _SendBlockInfoStatic()
		{
			const uint8_t *data_ptr = (const uint8_t *)&_block_info_static;
			const uint8_t data_length = sizeof(_block_info_static);
			
			_StaticInfoRequest(_block_info_static);
			_SendInChunks(0x72, data_ptr, data_length);
			
			return;
		}
		
		void _SendBlockInfoDynamic()
		{
			const uint8_t *data_ptr = (const uint8_t *)&_block_info_dynamic;
			const uint8_t data_length = sizeof(_block_info_dynamic);
			
			_DynamicInfoRequest(_block_info_dynamic);
			_SendInChunks(0x73, data_ptr, data_length);
			
			return;
		}
		
		void _SendError(uint8_t group, uint8_t code, uint16_t subcode)
		{
			error_t error = {};
			error.group = group;
			error.code = code;
			error.subcode = subcode;
			this->SendFrame((uint8_t *)&error, sizeof(error));
			
			return;
		}
		
		
		void _SendInChunks(const uint8_t fId, const uint8_t *data_ptr, const uint8_t data_length)
		{
			const uint8_t frame_size = 8;
			const uint8_t header_size = 2;
			const uint8_t payload_size = frame_size - header_size;
			
			uint8_t offset = 0;
			uint8_t page = 0;
			while(offset < data_length)
			{
				uint8_t buff[frame_size] = {fId, ++page};
				uint8_t buff_len = data_length - offset;
				if(buff_len > payload_size)
					buff_len = payload_size;
				memcpy(&buff[2], data_ptr + offset, buff_len);
				offset += payload_size;
				
				this->SendFrame(buff, buff_len + header_size);
			}
			
			return;
		}
		
		
		callback_static_info_request_t _StaticInfoRequest;
		callback_dynamic_info_request_t _DynamicInfoRequest;
		
		block_info_static_t _block_info_static = {};
		block_info_dynamic_t _block_info_dynamic = {};
		heartbeat_t _heartbeat = {};
		uint32_t _last_heartbeat = 0;
};
