#pragma once
#include <inttypes.h>
#include <string.h>
#include <MotorManager.h>
#include <MotorDeviceInterface.h>
#include "MotorFardriverNew_Data.h"
#include "MotorFardriverNew_Ctrl.h"

namespace FdNew = FardriverNew;
namespace MMD = MotorManagerData;

class MotorFardriverNew : public MotorDeviceInterface
{
	using callback_ready_t = void (*)(const FdNew::packet_raw_t *data);

	public:
		
		MotorFardriverNew() : MotorDeviceInterface(), _callback_ready(nullptr), 
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
				_manager->RawTx(_idx, FdNew::PacketInitTx, sizeof(FdNew::PacketInitTx));
				
				_need_auth = false;
			}
			
			// Если данные от контроллера приняты
			if(_ready == true)
			{
				_ready = false;
				_Barrier();		// _data читаем только после флага _ready

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
						
						// Авария - только биты ErrorAlarmMask (бит 15 ПО Fardriver аварией не считает)
						bool alarm = (packet->error_flags & FdNew::ErrorAlarmMask) != 0;
						if(alarm != _alarm)
						{
							_alarm = alarm;
							_error.code = (alarm == true) ? ERROR_CTRL : ERROR_NONE;
						}
						
						common_data->errors = packet->error_flags;
						common_data->rpm = packet->rpm / FdNew::RpmRatio;
						// 0.1 км/ч = об/мин x L[0.1 мм] x 60 / 1000000. Переполнение - выше ~11400 об/мин (колесо 500 мм - ~1000 км/ч)
						common_data->speed = (uint32_t)packet->rpm * _wheel_length * 6 / (100000UL * FdNew::RpmRatio);
						_OdometerUpdate(common_data->odometer, packet->rpm, FdNew::RpmRatio, FdNew::LinkTimeout, time);
						common_data->gear = GetGear(packet);
						common_data->roll = GetRoll(packet);
						
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
						
						// ток x0.25 А, напряжение x0.1 В -> мощность, Вт = I * U / 40
						int32_t power = ((int32_t)packet->current * (int32_t)packet->voltage) / 40;
						if(power > INT16_MAX) power = INT16_MAX;
						if(power < INT16_MIN) power = INT16_MIN;
						
						common_data->voltage = packet->voltage;
						common_data->current = ((int32_t)packet->current * 10) / 4;
						common_data->power = (int16_t)power;
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
					
					case 0x99:
					{
						FdNew::packet_99_t *packet = (FdNew::packet_99_t *) _data;
						
						common_data->throttle_voltage = packet->throttle_voltage;
						
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
				
				_Barrier();		// _data дочитан до того, как ISR получит право его перезаписать
				_busy = false;
			}

			// Если данные от контроллера не приняты
			else
			{
				// Если от контроллера давно не было данных
				if(_error.code != ERROR_LOST && time - _last_response_time > FdNew::LinkTimeout)
				{
					_manager->ResetCommonData(_idx);
					_error.code = ERROR_LOST;
					_alarm = false;
				}
			}
			
			if(time - _last_request_time > FdNew::PacketRequestInterval)
			{
				_last_request_time = time;

				_manager->RawTx(_idx, FdNew::PacketRequest, sizeof(FdNew::PacketRequest));
			}
			
			return;
		}
		
		// Приём пакета, вызывается из прерывания. Минимум действий.
		virtual void RawRx(const uint8_t *raw, const uint8_t length) override
		{
			if(length >= sizeof(FdNew::PacketInitRx) && memcmp(FdNew::PacketInitRx, raw, sizeof(FdNew::PacketInitRx)) == 0)
			{
				_need_auth = true; return;
			}
			if(_busy == true){ _error.code = ERROR_BUSY; return; }
			if(length != FdNew::PacketSize){ _error.code = ERROR_LENGTH; return; }
			if(memcmp(FdNew::PacketHeader, raw, sizeof(FdNew::PacketHeader)) != 0){ _error.code = ERROR_HEADER; return; }
			if(_CheckCRCSum(raw) == false){ _error.code = ERROR_CRC; return; }
			
			memcpy(_data, raw, sizeof(_data));
			_Barrier();		// _data записан до выставления флагов
			
			// Пакет исправен: сбрасываем ошибку канала, но не теряем аварию контроллера
			_error.code = (_alarm == true) ? ERROR_CTRL : ERROR_NONE;
			_busy = true;
			_ready = true;
			
			return;
		}
		
		// Команда контроллеру (MotorFardriverNew_Ctrl.h). false - нет такой команды, не то число параметров или неверные значения.
		template<typename... Args>
		bool SendCmd(FdNew::cmd_t cmd, Args... args)
		{
			const uint16_t p[] = { (uint16_t)args..., 0 };	// "+ 0": у команды без параметров массив не пустой
			
			return _SendCmd(cmd, p, sizeof...(Args));
		}
		
	private:
		
		// Передача: D0 биты 0-1
		static inline MMD::gear_t GetGear(const FdNew::packet_80_t *packet)
		{
			switch(packet->gear)
			{
				case FdNew::GEAR_RAW_NEUTRAL: { return MMD::GEAR_NEUTRAL; }
				case FdNew::GEAR_RAW_FORWARD: { return MMD::GEAR_FORWARD; }
				case FdNew::GEAR_RAW_REVERSE: { return MMD::GEAR_REVERSE; }
				case FdNew::GEAR_RAW_LOW:     { return MMD::GEAR_LOW; }
				default:                      { return MMD::GEAR_UNKNOWN; }
			}
		}
		
		// Направление вращения: D0 бит 5 - вращается, бит 4 - против направления передачи.
		// Передача R задаёт движение назад, N/D/Low - вперёд. Бит 6 (WEAK) и бит 7 к направлению не относятся.
		static inline MMD::roll_t GetRoll(const FdNew::packet_80_t *packet)
		{
			if(packet->moving == 0) return MMD::ROLL_STOP;
			
			bool forward = (packet->gear != FdNew::GEAR_RAW_REVERSE);
			if(packet->against == 1) forward = !forward;
			
			return (forward == true) ? MMD::ROLL_FORWARD : MMD::ROLL_REVERSE;
		}

		bool _CheckCRCSum(const uint8_t *array)
		{
			uint16_t crc = FdNew::CRC16(array, 14);
			
			return ( (crc & 0xFF) == array[14] && ((crc >> 8) & 0xFF) == array[15] );
		}
		
		// Найти команду, проверить число параметров, собрать пакет(ы) и отправить
		bool _SendCmd(FdNew::cmd_t cmd, const uint16_t *p, uint8_t count)
		{
			for(const FdNew::cmd_desc_t &desc : FdNew::Commands)
			{
				if(desc.cmd != cmd) continue;
				if(desc.args != count) return false;
				
				uint8_t buf[FdNew::PacketSize * 2];
				uint8_t length = desc.build(buf, p);
				if(length == 0) return false;
				
				_manager->RawTx(_idx, buf, length);
				
				return true;
			}
			
			return false;
		}
		
		void _PostProcessing()
		{
			return;
		}

		// Компиляторный барьер: запрещает переносить обращения к памяти (_data) через флаги.
		// На одноядерном Cortex-M3 этого достаточно, __DMB() не нужен.
		static inline void _Barrier()
		{
			__asm__ __volatile__("" ::: "memory");
		}

		callback_ready_t _callback_ready;	// Колбек принятого пакета, если указан
		uint32_t _last_request_time;		// Время последнего цикла обновления данных
		uint32_t _last_response_time;		// Время последнего принятого пакета данных
		volatile bool _need_auth;			// Флаг необходимости авторизации
		volatile bool _busy;				// Флаг того, что разбор данных не окончен и новые копировать нельзя
		volatile bool _ready;				// Флаг того, что массив данные приняты и готовы к анализу
		volatile bool _alarm;				// Контроллер сообщает об аварии (error_flags & ErrorAlarmMask)
		
		uint8_t _data[FdNew::PacketSize];	// Холодный массив данных (Работа в программе)

};
