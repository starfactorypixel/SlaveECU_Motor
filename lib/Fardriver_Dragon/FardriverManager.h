/*
	Библиотека построена на следующих идеологиях и допущениях:
	1. Пакет авторизации всегда начинается с "AT+";
	2. Пакет полезных данных всегда начинается с 0xAA;
*/

#pragma once

#include <inttypes.h>
#include <string.h>
#include "FardriverDriverInterface.h"
#include "Driver_2022/FardriverDriver_2022.h"
#include "Driver_2023/FardriverDriver_2023.h"

template <uint8_t _max_motors> 
class FardriverManager
{
	static constexpr uint8_t _rx_buff_size = 128;
	static constexpr uint8_t _rx_pa
	using tx_callback_t = void (*)(const uint8_t num, const uint8_t *buffer, uint8_t const length);
	
	public:
		
		FardriverManager(tx_callback_t tx_callback) : _tx_callback(tx_callback)
		{
			for(motor_t &motor : _motors)
			{
				// Init
			}
			
			return;
		}
		
		~FardriverManager()
		{
			for(motor_t &motor : _motors)
			{
				// De init
			}
			
			return;
		}
		
		/*
			Метод первочной обработки сырых данных. Вызывается в прерывании.
				uint8_t num - Номер мотора, начиная с 1.
				uint8_t *buffer - массив принятых данных.
				uint8_t length - Длина принятых данных.

				Функция реализована для работы со статическим буфером.
				Потом переделать на кольцевой, если нужно..
		*/
		void RXEvent(uint8_t num, uint8_t *buffer, uint8_t length)
		{
			if(--num >= _max_motors) return;
			
			motor_t &motor = _motors[num];
			
			// Если буфер переполнен, то караул..
			if(motor.rx_buff_write_idx + length >= _rx_buff_size) return;
			
			// Копируем новый кусок данных в буфер, склеивая его со старой головой, если она есть.
			memcpy(motor.rx_buff[motor.rx_buff_write_idx], buffer, length);
			motor.rx_buff_write_idx += length;
			
			// Проходимся по массиву и ищем там пакеты.
			bool allow_while = true;
			while(allow_while)
			{
				//uint8_t found_length = _SelectDriver();
				//if(found_length > 0)
				
				
				//motor.rx_buff[buff_start]
				//buff_length
				
				
				
				// Проверяем фрагмент на валидность и получаем длину валидных данных.
				int8_t payload_length = motor.driver->IsValidPacket( motor.rx_buff[motor.rx_buff_read_idx], (motor.rx_buff_write_idx - motor.rx_buff_read_idx) );
				switch(payload_length)
				{
					// Пакет не подошёл по длине.
					case -1:
					{
						allow_while = false;
						
						break;
					}
					// Пакет не подошёл по формату.
					case 0:
					{


						break;
					}
					// Пакет подошёл.
					default:
					{
						// Передаём этот фрагмент парсеру.
						motor.driver->ProcessPacket( motor.rx_buff[motor.rx_buff_read_idx], payload_length );
						
						// Обновляем индексы и смещения.
						motor.rx_buff_read_idx += payload_length;
						
						break;
					}
				}


				// Если всё прочитали, данных больше нет, хвостов нет.
				if(motor.rx_buff_write_idx - motor.rx_buff_read_idx == 0)
				{
					motor.rx_buff_write_idx = 0;
					motor.rx_buff_read_idx = 0;
					
					allow_while = false;
				}
			}
			
			return;
		}
		
		void Processing(uint32_t time)
		{
			if(time - _last_time < 1) return;
			
			for(motor_t &motor : _motors)
			{

			}
			
			_last_time = time;
			
			return;
		}

	private:
		
		void _SelectDriver(motor_t &motor)
		{
			// Если драйвер уже выбран, то нечего мучиться.
			if(motor.driver != nullptr) return 0;
			
			uint8_t result;
			uint8_t buff_length = motor.rx_buff_write_idx - motor.rx_buff_read_idx;
			
			// Пробуем старый драйвер
			delete motor.driver;
			motor.driver = new FardriverDriver_2022();
			result = motor.driver->IsValidPacket( motor.rx_buff[motor.rx_buff_read_idx], buff_length );
			if(result > 0) return result;
			
			// Пробуем новый драйвер
			delete motor.driver;
			motor.driver = new FardriverDriver_2023();
			result = motor.driver->IsValidPacket( motor.rx_buff[motor.rx_buff_read_idx], buff_length );
			if(result > 0) return result;
			
			/*
			// Используем драйвер-заглушку
			delete motor.driver;
			motor.driver = new FardriverDriverVoid();
			*/
			
			return 0;
		}

		struct motor_t
		{
			uint8_t rx_buff[_rx_buff_size];		// Приёмный буфер с сырыми данными.
			uint8_t rx_buff_write_idx;			// Индекс записи в приёмным буфер.
			uint8_t rx_buff_read_idx;			// Индекс чтения из приёмного буфера.

			FardriverDriverInterface driver;	// Драйвер.
		} _motors[_max_motors];

		uint32_t _last_time = 0;
		
};
