/*
	Класс работы с контрольными суммами пакетов UART контроллеров FarDriver:
		Nanjing FarDriver Controller Co., Ltd;
		SIAYQ FarDriver Controller;
	Отдельное спасибо Китайцам, которые зажопили протокол...
	
	Все методы с суффиксом '_old' реализуют расчёт CRC для старых контроллеров,
		у них ещё данные приходили в формате big-endian а CRC считалась суммированием,
		так-же все методы рассчитаны на обратный порядок байт, для исправления endian;
	Все методы с суффиксом '_new' реализуют расчёт CRC для новых контроллеров,
		у них формат CRC-16/MODBUS но с изменённым начальным значением и формат little-endian;
	
	Отдельное Спасибо за помощь:
		@maincraft
		@sergeyv
*/

#pragma once

#include <stdint.h>

class FardriverCRC
{
	public:
		
		/*
			Проверить исходящий массив на валидность CRC8 по старому протоколу.
			
			! Буфер данных должен быть инверсный !
		*/
		bool CheckTX_old(uint8_t *data)
		{
			uint8_t crc = GetCRC8_old(data);
			
			return (data[0] == ((~crc) & 0xFF) && data[1] == crc);
		}
		
		/*
			Проверить исходящий массив на валидность CRC8 по новому протоколу.
		*/
		bool CheckTX_new(uint8_t *data)
		{
			uint8_t crc = GetCRC8_new(data);
			
			return (data[6] == crc && data[7] == ((~crc) & 0xFF));
		}
		
		/*
			Проверить входящий массив на валидность CRC16 по старому протоколу.
			
			! Буфер данных должен быть инверсный !
		*/
		bool CheckRX_old(uint8_t *data)
		{
			uint16_t crc = GetCRC16_old(data);
			
			return (data[0] == (crc & 0xFF) && data[1] == ((crc >> 8) & 0xFF));
		}
		
		/*
			Проверить входящий массив на валидность CRC16 по новому протоколу.
		*/
		bool CheckRX_new(uint8_t *data)
		{
			uint16_t crc = GetCRC16_new(data);
			
			return (data[14] == (crc & 0xFF) && data[15] == ((crc >> 8) & 0xFF));
		}
		
		
		/*
			Вставить в исходящий массив CRC8 по старому протоколу.
			
			! Буфер данных должен быть инверсный !
		*/
		void PutTX_old(uint8_t *data)
		{
			uint8_t crc = GetCRC8_old(data);
			data[0] = ~crc;
			data[1] = crc;
			
			return;
		}
		
		/*
			Вставить в исходящий массив CRC8 по новому протоколу.
		*/
		void PutTX_new(uint8_t *data)
		{
			uint8_t crc = GetCRC8_new(data);
			data[6] = crc;
			data[7] = ~crc;
			
			return;
		}
		
		/*
			Вставить в входящий массив CRC16 по старому протоколу.
			
			! Буфер данных должен быть инверсный !
		*/
		void PutRX_old(uint8_t *data)
		{
			uint16_t crc = GetCRC16_old(data);
			data[0] = (crc & 0xFF);
			data[1] = ((crc >> 8) & 0xFF);
			
			return;
		}
		
		/*
			Вставить в входящий массив CRC16 по новому протоколу.
		*/
		void PutRX_new(uint8_t *data)
		{
			uint16_t crc = GetCRC16_new(data);
			data[14] = (crc & 0xFF);
			data[15] = ((crc >> 8) & 0xFF);
			
			return;
		}
		
		
		/*
			Расчёт CRC8 для старых контроллеров.
			Параметры: Алгебраическая сумма байт
			
			! Буфер данных должен быть инверсный !
		*/
		uint8_t GetCRC8_old(uint8_t *buffer)
		{
			uint8_t crc = 0x00;
			
			for(uint8_t i = 2; i < 8; ++i)
			{
				crc += buffer[i];
			}
			
			return crc;
		}
		
		/*
			Расчёт CRC8 для новых контроллеров.
			Параметры: Алгебраическая сумма байт
		*/
		uint8_t GetCRC8_new(uint8_t *buffer)
		{
			uint8_t crc = 0x00;
			
			for(uint8_t i = 0; i < 6; ++i)
			{
				crc += buffer[i];
			}
			
			return crc;
		}
		
		/*
			Расчёт CRC16 для старых контроллеров.
			Параметры: Алгебраическая сумма байт

			! Буфер данных должен быть инверсный !
		*/
		uint16_t GetCRC16_old(uint8_t *buffer)
		{
			uint16_t crc = 0x0000;
			
			for(uint8_t idx = 2; idx < 16; ++idx)
			{
				crc += buffer[idx];
			}
			
			return crc;
		}
		
		/*
			Расчёт CRC16 для новых контроллеров.
			Параметры: Poly: 0x8005, Init: 0x7F3C, RefIn: true, RefOut: true, XorOut: false
		*/
		uint16_t GetCRC16_new(uint8_t *buffer)
		{
			uint16_t crc = 0x7F3C;
			
			for(uint8_t idx = 0; idx < 14; ++idx)
			{
				crc ^= (uint16_t)buffer[idx];
				for(uint8_t i = 8; i != 0; --i)
				{
					if( (crc & 0x0001) != 0)
					{
						crc >>= 1;
						crc ^= 0xA001;
					}
					else
					{
						crc >>= 1;
					}
				}
			}
			
			return crc;
		}
};
