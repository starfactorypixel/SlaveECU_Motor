/*
	Класс парсинга пакетов UART контроллеров FarDriver.
		Nanjing FarDriver Controller Co., Ltd.
		SIAYQ FarDriver Controller.
	Отдельное спасибо Китайцам, которые зажопили протокол...
*/

#pragma once

#include "MotorErrors.h"
#include "MotorPackets.h"

template <uint8_t _motor_idx> 
class FardriverController
{
	static const uint16_t _rx_buffer_timeout = 20;	// Время мс сброса принимаемого пакета.
	static const uint8_t _rx_buffer_size = 16;		// Общий размер пакета.
	
	using event_callback_t = void(*)(uint8_t motor_idx, uint8_t data[_rx_buffer_size]);
	using error_callback_t = void(*)(uint8_t motor_idx, motor_error_t code);
	
	public:
		
		FardriverController()
		{
			_ClearBuff();

			return;
		}
		
		/*
			Регистрирует колбек, который возвращает принятый пакет.
		*/
		void SetEventCallback(event_callback_t callback)
		{
			_event_callback = callback;
			
			return;
		}
		
		/*
			Регистрирует колбек, который возвращает принятый флаг(и) ошибок.
		*/
		void SetErrorCallback(error_callback_t callback)
		{
			_error_callback = callback;

			return;
		}
		
		/*
			(Interrupt) Вставка принятого байта и обработка пакета.
		*/
		void RXByte(uint8_t data, uint32_t time)
		{
			// Если с момента последнего байта прошло более _packet_timeout мс, то чистим буфер.
			if(time - _rx_buffer_timeout > _rx_buffer_last_time)
			{
				_ClearBuff();
			}
			
			// Если есть место для нового байта, то вставляем его.
			if(_rx_buffer_idx < _rx_buffer_size)
			{
				_rx_buffer_last_time = time;

				_rx_buffer[_rx_buffer_idx++] = data;
			}
			
			// Если размер принятых байт равен _packet_length_total, то проверяем пакет.
			if(_rx_buffer_idx == _rx_buffer_size)
			{
				if( _CheckBuffFormat() == true )
				{
					memcpy(&_work_buffer, _rx_buffer, _rx_buffer_size);
					_work_buffer_ready = true;
				}
				
				// Нету смысла чистить тут, потому что через паузу буфер будет очищен автматически.
				//_ClearBuff();
			}
			
			return;
		}
		
		/*
			Обработка принытых данных.
			Вызываться должна с интервалом, не более 30 мс!
		*/
		void Processing(uint32_t time)
		{
			if(_work_buffer_ready == true)
			{
				
				// Вычитываем ошибки, и вызываем колбек, если нужно.
				if(_work_buffer[1] == 0x00 && _error_callback != nullptr)
				{
					packet_0_t *packet = (packet_0_t*) &_work_buffer;
					if(packet->ErrorFlags != _lastErrorFlags)
					{
						_lastErrorFlags = packet->ErrorFlags;
						
						_error_callback(_motor_idx, packet->ErrorFlags);
					}
				}
				
				// Вызываем колбек события, если он зарегистрирован.
				if(_event_callback != nullptr)
				{
					_event_callback(_motor_idx, _work_buffer);
				}
				
				_work_buffer_ready = false;
				
			}
			
			return;
		}
		
	private:

		/*
			Проверяет принятый пакет на валидность.
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
			
			for(uint8_t i = 0; i < (_rx_buffer_size - 2); ++i)
			{
				result += _rx_buffer[i];
			}
			
			return result;
		}
		
		/*
			Очищает буфер.
		*/
		void _ClearBuff()
		{
			memset(&_rx_buffer, 0x00, _rx_buffer_size);
			_rx_buffer_idx = 0;
			
			return;
		}
		
		
		event_callback_t _event_callback = nullptr;
		error_callback_t _error_callback = nullptr;
		
		uint8_t _rx_buffer[_rx_buffer_size];	// Приёмный (горячий) буфер.
		uint8_t _rx_buffer_idx;					// Индекс принимаего байта.
		uint32_t _rx_buffer_last_time;			// Время мс последнего принятого байта.

		uint8_t _work_buffer[_rx_buffer_size];	// Рабочий (холодный) буфер.
		bool _work_buffer_ready = false;		// Готовность рабочего буфера.
		
		uint16_t _lastErrorFlags = 0x0000;
		
};
