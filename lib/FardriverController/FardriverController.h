/*
	Класс парсинга пакетов UART контроллеров FarDriver.
		Nanjing FarDriver Controller Co., Ltd.
		SIAYQ FarDriver Controller.
	Отдельное спасибо Китайцам, которые зажопили протокол...
*/

#pragma once

#include "MotorErrors.h"
#include "MotorPackets.h"

template <uint8_t _motor_idx = 0> 
class FardriverController
{
	static const uint16_t _rx_buffer_timeout = 10;	// Время мс до сброса принимаемого пакета.
	static const uint8_t _rx_buffer_size = 16;		// Общий размер пакета.
	static const uint16_t _request_time = 550;		// Интервал отправки запраса данных в контроллер.
	static const uint16_t _unactive_timeout = 500;	// Время мс бездейтсвия, после которого считается что связи с контроллером нет.
	
	using event_callback_t = void(*)(const uint8_t motor_idx, const uint8_t data[_rx_buffer_size]);
	using error_callback_t = void(*)(const uint8_t motor_idx, const motor_error_t code);
	using tx_callback_t = void(*)(const uint8_t motor_idx, const uint8_t *data, const uint8_t data_len);
	
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
			Регистрирует колбек, который отправляет данные контроллеру.
		*/
		void SetTXCallback(tx_callback_t callback)
		{
			_tx_callback = callback;
			
			return;
		}
		
		/*
			(Interrupt) Вставка принятого байта и обработка пакета.
		*/
		void RXByte(uint8_t data, uint32_t time)
		{
			// Если с момента последнего байта прошло более _packet_timeout мс.
			if(time - _rx_buffer_timeout > _rx_buffer_last_time)
			{
				_ClearBuff();
			}
			
			// Если есть место для нового байта.
			if(_rx_buffer_idx < _rx_buffer_size)
			{
				_rx_buffer_last_time = time;

				_rx_buffer[_rx_buffer_idx++] = data;
			}
			
			// Если размер принятых байт равен _rx_buffer_size.
			if(_rx_buffer_idx == _rx_buffer_size)
			{
				if( _CheckBuffFormat() == true )
				{
					memcpy(&_work_buffer, _rx_buffer, _rx_buffer_size);
					_work_buffer_ready = true;
					_isActive = true;
				}
				
				// Нету смысла чистить тут, потому что через паузу буфер будет очищен автматически.
				//_ClearBuff();
			}
			
			return;
		}
		
		/*
			Флаг активного соединенеия с контроллером.
		*/
		bool IsActive()
		{
			return _isActive;
		}
		
		/*
			Обработка принытых данных.
			Вызываться должна с интервалом, не более 30 мс!
		*/
		void Processing(uint32_t time)
		{
			// Нужно ответить на запрос авторизации.
			if(_need_init_tx == true)
			{
				_tx_callback(_motor_idx, motor_packet_init_tx, sizeof(motor_packet_init_tx));
				
				_need_init_tx = false;
			}
			
			// Каждые _request_time отправляет запросы в контроллер.
			if(time - _request_last_time > _request_time)
			{
				_request_last_time = time;
				
				_tx_callback(_motor_idx, motor_packet_request_tx, sizeof(motor_packet_request_tx));
			}
			
			// Обработка принятого пакета.
			if(_work_buffer_ready == true)
			{
				// Вычитываем ошибки, и вызываем колбек, если нужно.
				if(_work_buffer[1] == 0x00 && _error_callback != nullptr)
				{
					motor_packet_0_t *packet = (motor_packet_0_t*) &_work_buffer;
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
			
			// Время послденего байта больше _unactive_timeout.
			if(time - _rx_buffer_last_time > _unactive_timeout)
			{
				_isActive = false;
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
			
			// Если приняли 'нормальный' пакет.
			if(_rx_buffer[0] == 0xAA)
			{
				motor_packet_raw_t *raw = (motor_packet_raw_t*) &_rx_buffer;
				
				if( _GetBuffCRC() == raw->_end.crc )
				{
					result = true;
				}
			}
			// Если приняли пакет авторизации.
			else if( memcmp(&motor_packet_init_rx, &_rx_buffer, _rx_buffer_size) == 0 )
			{
				_need_init_tx = true;
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
		tx_callback_t _tx_callback = nullptr;
		
		uint8_t _rx_buffer[_rx_buffer_size];	// Приёмный (горячий) буфер.
		uint8_t _rx_buffer_idx;					// Индекс принимаего байта.
		uint32_t _rx_buffer_last_time = 0;		// Время мс последнего принятого байта.

		uint8_t _work_buffer[_rx_buffer_size];	// Рабочий (холодный) буфер.
		bool _work_buffer_ready = false;		// Готовность рабочего буфера.
		
		uint16_t _lastErrorFlags = 0x0000;

		bool _need_init_tx = false;

		uint32_t _request_last_time = 0;

		bool _isActive;

		//enum state_t : uint8_t {STATE_IDLE, STATE_AUTH, STATE_WORK, STATE_LOST} _state = STATE_IDLE;
		
};
