/*

*/

#pragma once

#include "MotorPackets.h"
#include "MotorErrors.h"

template <uint8_t _motor_num, uint8_t _rx_buffer_size = 32> 
class FardriverController
{
	static const uint16_t _packet_timeout = 20;		// Время мс сброса принимаемого пакета.
	static const uint8_t _packet_length_total = 16;	// Общий размер пакета.
	static const uint8_t _packet_length_crc = 14;	// Размер пакета до CRC.
	
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
		
		void RXByte(uint8_t data, uint32_t time)
		{
			if(time - _packet_timeout > _rx_last_time)
			{
				_ClearBuff();
			}

			if(_rx_buffer_idx < _rx_buffer_size)
			{
				_rx_last_time = time;

				_rx_buffer[_rx_buffer_idx++] = data;
			}
			
			return;
		}
		
		void Processing(uint32_t time)
		{
			if(_rx_buffer_idx == _packet_length_total)
			{
				if( _CheckBuffFormat() == true )
				{

				}
				else
				{
					_ClearBuff();
				}
			}


			if(false)
			{
				_error_callback(_motor_num, CONTROLLER_OVER_TEMP);
			}

			//_CheckBuffFormat();
		}
		
	private:

		/*
			Проверяем принчтый пакет на валидность.
		*/
		inline bool _CheckBuffFormat()
		{
			bool result = false;
			
			if(_rx_buffer[0] == 0xAA)
			{
				packet_raw_t *raw = (packet_raw_t*) &_rx_buffer;
				
				if( _GetBuffCRC() == raw->_end.crc )
				{
					result = true;
				}
			}
			
			return result;
		}

		/*
			Расчитывает CRC принятого пакета.
		*/
		inline uint16_t _GetBuffCRC()
		{
			uint16_t result = 0x0000;
			
			for(uint8_t i = 0; i < _packet_length_crc; ++i)
			{
				result += _rx_buffer[i];
			}
			
			return result;
		}
		
		void _ClearBuff()
		{
			memset(&_rx_buffer, 0x00, _rx_buffer_size);
			_rx_buffer_idx = 0;
			_rx_buffer_crc_accum = 0;
			
			return;
		}

		error_callback_t _error_callback;

		uint8_t _rx_buffer[_rx_buffer_size];	// Приёмный буфер.
		uint8_t _rx_buffer_idx;					// Индекс принимаего байта.
		uint16_t _rx_buffer_crc_accum;			// Аккумулятор подстчёта CRC.
		uint32_t _rx_last_time;					// Время мс последнего принятого байта.
		

		uint16_t _lastErrorFlags = 0x0000;


};
