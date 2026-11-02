#pragma once
#include <inttypes.h>
#include <string.h>
#include <CUtils_String.h>
#include <MotorManager.h>
#include <MotorDeviceInterface.h>
#include "MotorFardriverOld_Data.h"

namespace FdOld = FardriverOld;
namespace MMD = MotorData;

class MotorFardriverOld : public MotorDeviceInterface
{
	using callback_ready_t = void (*)(const FdOld::packet_raw_t *data);
	
	public:
		
		MotorFardriverOld() : MotorDeviceInterface(), _callback_ready(nullptr), 
			_last_request_time(0), _last_response_time(0), _need_auth(false), 
			_busy(false), _ready(false), _alarm(false)
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
				_manager->RawTx(_idx, FdOld::PacketInitTx, sizeof(FdOld::PacketInitTx));
				
				_need_auth = false;
			}
			
			// Объект-хранилище данных мотора
			MotorData::MotorDataObj &motor_data = _manager->motor_data[_idx];

			// Если данные от контроллера приняты
			if(_ready == true)
			{
				_ready = false;
				_Barrier();		// _data читаем только после флага _ready

				_PostProcessing();

				// Напихиваем полезные данные в общий объект
				FdOld::packet_raw_t *raw = (FdOld::packet_raw_t *) _data;

				switch(raw->_A1)
				{
					case 0x00:
					{
						FdOld::packet_00_t *packet = (FdOld::packet_00_t *) _data;
						
						bool alarm = (packet->error_flags != 0);
						if(alarm != _alarm)
						{
							_alarm = alarm;
							_error.code = (alarm == true) ? ERROR_CTRL : ERROR_NONE;
						}
						
						motor_data.data.errors = packet->error_flags;
						motor_data.data.rpm = packet->rpm / FdOld::RpmRatio;
						// 0.1 км/ч = об/мин x L[0.1 мм] x 60 / 1000000. Переполнение - выше ~11400 об/мин (колесо 500 мм - ~1000 км/ч)
						motor_data.data.speed = (uint32_t)packet->rpm * _wheel_length * 6 / (100000UL * FdOld::RpmRatio);
						_OdometerUpdate(motor_data.data.odometer, packet->rpm, FdOld::RpmRatio, FdOld::LinkTimeout, time);
						motor_data.data.gear = FixGear(packet->gear);
						motor_data.data.roll = FixRoll(packet->gear, packet->roll);
						
						break;
					}
					case 0x01:
					{
						FdOld::packet_01_t *packet = (FdOld::packet_01_t *) _data;
						
						// ток x0.25 А, напряжение x0.1 В -> мощность, Вт = I * U / 40
						int32_t power = ((int32_t)packet->current * (int32_t)packet->voltage) / 40;
						if(power > INT16_MAX) power = INT16_MAX;
						if(power < INT16_MIN) power = INT16_MIN;
						
						motor_data.data.voltage = packet->voltage;
						motor_data.data.current = ((int32_t)packet->current * 10) / 4;
						motor_data.data.power = (int16_t)power;
						motor_data.data.throttle = packet->throttle;
						
						break;
					}
					case 0x04:
					{
						FdOld::packet_04_t *packet = (FdOld::packet_04_t *) _data;
						
						motor_data.data.temp_controller = FixTemp(packet->temp_controller);
						
						break;
					}
					case 0x0D:
					{
						FdOld::packet_0D_t *packet = (FdOld::packet_0D_t *) _data;
						
						motor_data.data.temp_motor = FixTemp(packet->temp_motor);
						
						break;
					}
					case 0x0C:
					{
						// Пока не получим первый раз последний пакет посылки от контроллера (0x0C) - считаем что данные не готовы.
						// На самом деле это предпоследний пакет посылки, но последний это 0x00 и его сложно отследить.
						// В последствии считаем что данные актуальны, хоть и опаздывают.
						motor_data.Ready(true);

						break;
					}
				}

				_last_response_time = time;
				
				if(_callback_ready != nullptr)
				{
					_callback_ready(raw);
				}
				
				_Barrier();		// _data дочитан до того, как ISR получит право его перезаписать
				_busy = false;
			}
			
			// Если данные от контроллера не приняты
			else
			{
				// Если от контроллера давно не было данных
				if(_error.code != ERROR_LOST && time - _last_response_time > FdOld::LinkTimeout)
				{
					motor_data.Clear();
					_error.code = ERROR_LOST;
					_alarm = false;
				}
			}

			if(time - _last_request_time > FdOld::PacketRequestInterval)
			{
				_last_request_time = time;

				_manager->RawTx(_idx, FdOld::PacketRequest, sizeof(FdOld::PacketRequest));
			}
			
			return;
		}
		
		// Приём пакета, вызывается из прерывания. Минимум действий.
		virtual void RawRx(const uint8_t *raw, const uint8_t length) override
		{
			if(length >= sizeof(FdOld::PacketInitRx) && memcmp(FdOld::PacketInitRx, raw, sizeof(FdOld::PacketInitRx)) == 0)
			{
				_need_auth = true; return;
			}
			if(_busy == true){ _error.code = ERROR_BUSY; return; }
			if(length != FdOld::PacketSize){ _error.code = ERROR_LENGTH; return; }
			if(memcmp(FdOld::PacketHeader, raw, sizeof(FdOld::PacketHeader)) != 0){ _error.code = ERROR_HEADER; return; }
			if(_CheckCRCSum(raw) == false){ _error.code = ERROR_CRC; return; }
			
			memcpy(_data, raw, sizeof(_data));
			_Barrier();		// _data записан до выставления флагов
			
			// Пакет исправен: сбрасываем ошибку канала, но не теряем аварию контроллера
			_error.code = (_alarm == true) ? ERROR_CTRL : ERROR_NONE;
			_busy = true;
			_ready = true;
			
			return;
		}
		
		static inline int16_t FixTemp(uint8_t raw)
		{
			return (raw <= 200) ? (uint8_t)raw : (int8_t)raw;
		}
		
		// Передача: младший полубайт, биты 0-1 (биты 2-3 - уровень скорости). 0x0C - N, 0x01 - D, 0x0E - R.
		static inline MMD::gear_t FixGear(uint8_t raw)
		{
			switch(raw & 0x03)
			{
				case 0x00: { return MMD::GEAR_NEUTRAL; }
				case 0x01: { return MMD::GEAR_FORWARD; }
				case 0x02: { return MMD::GEAR_REVERSE; }
				default:   { return MMD::GEAR_LOW; }
			}
		}
		
		// Направление: старший полубайт - 0 стоит, 1 вращается в сторону передачи, 2 и больше - против.
		// Передача R задаёт движение назад, N/D/Low - вперёд (как в ПО Fardriver).
		static inline MMD::roll_t FixRoll(uint8_t gear, uint8_t raw)
		{
			if(raw == 0) return MMD::ROLL_STOP;
			
			bool forward = ((gear & 0x03) != 0x02);
			if(raw >= 2) forward = !forward;
			
			return (forward == true) ? MMD::ROLL_FORWARD : MMD::ROLL_REVERSE;
		}
		
	private:
		
		// Компиляторный барьер: запрещает переносить обращения к памяти (_data) через флаги.
		static inline void _Barrier()
		{
			__asm__ __volatile__("" ::: "memory");
		}
		
		bool _CheckCRCSum(const uint8_t *array)
		{
			uint16_t crc = 0x0000;
			
			for(uint8_t i = 0; i < 14; ++i) { crc += array[i]; }
			
			return ( ((crc >> 8) & 0xFF) == array[14] && (crc & 0xFF) == array[15] );
		}
		
		void _PostProcessing()
		{
			// Т.к. данные представлены в формате big-endian, обращаем массив для простоты работы
			array_reverse(_data, sizeof(_data));
			
			return;
		}

		callback_ready_t _callback_ready;	// Колбек принятого пакета, если указан
		uint32_t _last_request_time;		// Время последнего цикла обновления данных
		uint32_t _last_response_time;		// Время последнего принятого пакета данных
		volatile bool _need_auth;			// Флаг необходимости авторизации
		volatile bool _busy;				// Флаг того, что разбор данных не окончен и новые копировать нельзя
		volatile bool _ready;				// Флаг того, что массив данных принят и готов к анализу
		volatile bool _alarm;				// Контроллер сообщает об аварии
		
		uint8_t _data[FdOld::PacketSize];	// Копия принятого пакета для разбора в Tick
		
};
