#pragma once
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <MotorManager.h>
#include <MotorDeviceInterface.h>
#include "MotorFardriverNew_Data.h"

namespace FdNew = FardriverNew;
namespace MMD = MotorManagerData;

class MotorFardriverNew : public MotorDeviceInterface
{
	using callback_ready_t = void (*)(const FdNew::packet_raw_t *data);
	
	public:
		
		MotorFardriverNew() : MotorDeviceInterface(), _callback_ready(nullptr), 
			_last_request_time(0), _last_response_time(0), _auth(false), _need_auth(false), 
			_busy(false), _ready(false)
		{
			memset(_data, 0x00, sizeof(_data));
			
			return;
		}
		
		void SetReadyCallback(callback_ready_t ready)
		{
			_callback_ready = ready;
			
			return;
		}
		
		virtual void Init() override
		{
			return;
		}
		
		virtual void Tick(uint32_t &time) override
		{
			if(_need_auth == true)
			{
				_manager->RawTx(_idx, FdNew::PacketInitTx, sizeof(FdNew::PacketInitTx));
				
				_need_auth = false;
			}
			
			// Если данные от контроллера приняты
			if(_ready == true)
			{
				_ready = false;

				_PostProcessing();

				// Напихиваем полезные данные в общий объект
				FdNew::packet_raw_t *raw = (FdNew::packet_raw_t *) _data;
				MMD::common_data_t *common_data = &_manager->common_data[_idx];
				
				switch(raw->_A1)
				{
					case 0x80:
					case 0x87:
					case 0x8E:
					case 0x95:
					case 0x9C:
					case 0xA3:
					case 0xAA:
					case 0xB0:
					{
						FdNew::packet_80_t *packet = (FdNew::packet_80_t *) _data;
						
						if(common_data->errors != packet->error_flags)
						{
							_error.code = (packet->error_flags > 0) ? ERROR_CTRL : ERROR_NONE;
						}
						
						common_data->errors = packet->error_flags;
						common_data->rpm = packet->rpm >> 2;
						common_data->speed = ((_speed_coefficient * packet->rpm) / 400000UL);
						common_data->gear = FixGear(packet->gear);
						common_data->roll = FixRoll(packet->roll);
						
						break;
					}
					
					case 0x81:
					case 0x88:
					case 0x8F:
					case 0x96:
					case 0x9D:
					case 0xA4:
					case 0xAB:
					case 0xB1:
					{
						FdNew::packet_81_t *packet = (FdNew::packet_81_t *) _data;
						
						int16_t power = (((uint32_t)abs(packet->current) * (uint32_t)packet->voltage) / 40U);
						if(packet->current < 0) power = -power;
						
						common_data->voltage = packet->voltage;
						common_data->current = ((packet->current * 10U) / 4U);
						common_data->power = power;
						common_data->throttle = packet->throttle;
						
						break;
					}

					case 0x82:
					case 0x89:
					case 0x90:
					case 0x97:
					case 0x9E:
					case 0xA5:
					case 0xAC:
					case 0xB2:
					{
						break;
					}
					
					case 0xB3:
					{
						FdNew::packet_B3_t *packet = (FdNew::packet_B3_t *) _data;
						
						common_data->temp_controller = packet->temp_controller;
						
						break;
					}
					
					case 0xB5:
					{
						FdNew::packet_B5_t *packet = (FdNew::packet_B5_t *) _data;
						
						common_data->temp_motor = packet->temp_motor;
						
						break;
					}
					
					case 0xB6:
					{
						// Пока не получим первый раз последний пакет в посылки от контроллера (0xB6) - считаем что данные не готовы.
						// В последствии считаем что данные актуальны, хоть и опаздывают.
						_manager->common_data_ready[_idx] = true;
						
						break;
					}

					default:
					{
						break;
					}
				}

				_last_response_time = time;
				
				if(_callback_ready != nullptr)
				{
					_callback_ready(raw);
				}
				
				_busy = false;
			}

			// Если данные от контроллера не приняты
			else
			{
				// Если от контроллера давно не было данных
				if(_error.code != ERROR_LOST && time - _last_response_time > 1000)
				{
					_manager->ResetCommonData(_idx);
					_error.code = ERROR_LOST;
				}
			}
			
			if(time - _last_request_time > FdNew::PacketRequestInerval /*&& _auth == true*/)
			{
				_last_request_time = time;

				_manager->RawTx(_idx, FdNew::PacketRequest, sizeof(FdNew::PacketRequest));
			}
			
			return;
		}
		
		// Приём пакета, в прерывании. Минимум самый важный действий.
		virtual void RawRx(const uint8_t *raw, const uint8_t length) override
		{
			_error.code = ERROR_NONE;
			if(memcmp(FdNew::PacketInitRx, raw, sizeof(FdNew::PacketInitRx)) == 0)
			{
				_need_auth = true; return;
			}
			if(_busy == true){ _error.code = ERROR_BUSY; return; }
			if(length != FdNew::PacketSize){ _error.code = ERROR_LENGTH; return; }
			if(memcmp(FdNew::PacketHeader, raw, sizeof(FdNew::PacketHeader)) != 0){ _error.code = ERROR_HEADER; return; }
			if(_CheckCRCSum(raw) == false){ _error.code = ERROR_CRC; return; }
			
			memcpy(_data, raw, sizeof(_data));
			
			_ready = true;
			_busy = true;
			_auth = true;
			
			return;
		}
		
		static inline MMD::gear_t FixGear(uint8_t raw)
		{
			// 0С - 00 - Нейтраль
			// 01 - 01 - Передняя
			// 0E - 02 - Задняя
			
			MMD::gear_t gear;
			switch(raw)
			{
				case 0x0C: { gear = MMD::GEAR_NEUTRAL; break; }
				case 0x01: { gear = MMD::GEAR_FORWARD; break; }
				case 0x0E: { gear = MMD::GEAR_REVERSE; break; }
				default:   { gear = MMD::GEAR_UNKNOWN; break; }
			}
			
			return gear;
		}
		
		static inline MMD::roll_t FixRoll(uint8_t raw)
		{
			// 01 - Стоп
			// 02 - Вперёд
			// 03 - Назад
			
			MMD::roll_t roll;
			switch(raw)
			{
				case 0x01: { roll = MMD::ROLL_STOP; break; }
				case 0x03: { roll = MMD::ROLL_REVERSE; break; }
				case 0x02: { roll = MMD::ROLL_FORWARD; break; }
				default:   { roll = MMD::ROLL_UNKNOWN; break; }
			}
			
			return roll;
		}
		
	private:
		
		bool _CheckCRCSum(const uint8_t *array)
		{
			uint16_t crc = 0x7F3C;
			
			for(uint8_t idx = 0; idx < 14; ++idx)
			{
				crc ^= (uint16_t)array[idx];
				for(uint8_t i = 8; i != 0; --i)
				{
					if((crc & 0x0001) != 0)
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
			
			return ( (crc & 0xFF) == array[14] && ((crc >> 8) & 0xFF) == array[15] );
		}
		
		void _PostProcessing()
		{
			return;
		}

		callback_ready_t _callback_ready;	// Колбек принятого пакета, если указан
		uint32_t _last_request_time;		// Время последнего цикла обновления данных
		uint32_t _last_response_time;		// Время последнего принятого пакета данных
		bool _auth;							// Флаг пройденной авторизации
		bool _need_auth;					// Флаг необходимости авторизации
		bool _busy;							// Флаг того, что разбор данных не окончен и новые копировать нельзя
		bool _ready;						// Флаг того, что массив данные приняты и готовы к анализу
		
		uint8_t _data[FdNew::PacketSize];	// Холодный массив данных (Работа в программе)

};
