/*

*/

#pragma once

#include "MotorPackets.h"
#include "MotorErrors.h"

template <uint8_t _motor_num, uint8_t _rx_buff_size = 32> 
class FardriverController
{
	using error_callback_t = void(*)(uint8_t num, motor_error_t code);
	
	public:
		FardriverController()
		{
			_ClearBuff();

			return;
		}

		void SetErrorCallback(error_callback_t callback)
		{
			_error_callback = callback;

			return;
		}
		
		void Processing(uint32_t time)
		{
			if(false)
			{
				_error_callback(_motor_num, CONTROLLER_OVER_TEMP);
			}

			_CheckBuff();
		}
		
	private:

		bool _CheckBuff()
		{
			bool result = false;

			if(_rx_buffer[0] == 0xAA && _rx_buffer_idx == 16)
			{
				packet_raw_t *raw = (packet_raw_t*) &_rx_buffer;
				
				// Перенести подсчёт CRC на этап приёма байта.
				uint16_t crc = 0x0000;
				for(uint8_t i = 0; i < (_rx_buffer_idx - 2); ++i)
				{
					crc += _rx_buffer[i];
				}

				if(crc == raw->_end.crc)
				{
					result = true;
				}
			}

			return result;
		}
		
		void _ClearBuff()
		{
			memset(&_rx_buffer, 0x00, sizeof(_rx_buffer));
			_rx_buffer_idx = 0;

			return;
		}

		error_callback_t _error_callback;

		uint8_t _rx_buffer[_rx_buff_size];
		uint8_t _rx_buffer_idx;


};
