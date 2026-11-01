#pragma once
#include <inttypes.h>

class MotorManager;

class MotorDeviceInterface
{
	public:
		
		enum error_code_t : int8_t
		{
			ERROR_NONE = 0,			// Ошибок нет
			ERROR_BUSY = -1,		// Новый пакет пришёл раньше чем данные были обработаны
			ERROR_LENGTH = -2,		// Новый пакет не верной длинны
			ERROR_HEADER = -3,		// Новый пакет имеет неверный заголовок
			ERROR_CRC = -4,			// Новый пакет не прошёл проверку CRC
			ERROR_AUTH = -5,		// Пакет авторизации не соответствует контроллеру
			ERROR_CTRL = 1,			// Получен код ошибки от контроллера
			ERROR_LOST = 2,			// Потеря связи с контроллером
		};
		
		
		MotorDeviceInterface() : _initiated(false), _manager(nullptr), _idx(0), _wheel_length(0), _odometer_step(0), _odometer_time(0), _odometer_acc(0)
		{

		}
		
		virtual void Init() = 0;
		virtual void Tick(uint32_t &time) = 0;
		virtual void RawRx(const uint8_t *raw, const uint8_t length) = 0;
		
		// Диаметр колеса, мм: длина окружности для скорости и шаг одометра. Вызывается при запуске.
		void SetWheelDiameter(uint16_t mm)
		{
			// Длина окружности, 0.1 мм = pi * D * 10
			_wheel_length = ((uint32_t)mm * 314159UL + 5000UL) / 10000UL;
			
			// Одометр копит об/мин x мс. 100 м = 1000000 / L оборотов, один оборот = 60000 об/мин x мс
			_odometer_step = (_wheel_length == 0) ? 0 : (uint32_t)(1000000ULL * 60000ULL / _wheel_length);
			
			return;
		}
		
		bool IsInitiated()
		{
			return _initiated;
		}
		
		void PrepareInit(MotorManager *obj, uint8_t idx)
		{
			_initiated = true;
			_manager = obj;
			_idx = idx;
		}

		bool GetNewError(error_code_t &code)
		{
			code = _error.code;
			if(code != _error.prev)
			{
				_error.prev = code;
				return true;
			}
			
			return false;
		}
	
	protected:

		// Одометр: драйвер вызывает на каждом пакете с оборотами, odometer - common_data_t::odometer (100 м).
		// Только прибавляет: начальное значение кладёт пользователь.
		// rpm - как отдаёт контроллер (об/мин x ratio). max_dt - интервал, мс, больше которого путь не считаем
		// (первый пакет или связь пропадала), обычно LinkTimeout драйвера; до ~30000 накопитель в uint32 не переполнится.
		void _OdometerUpdate(uint32_t &odometer, uint16_t rpm, uint8_t ratio, uint16_t max_dt, uint32_t time)
		{
			uint32_t dt = time - _odometer_time;
			_odometer_time = time;
			if(dt > max_dt || _odometer_step == 0) return;
			
			// Копим обороты (об/мин x ratio x мс), каждые 100 м пути -> odometer++
			uint32_t step = _odometer_step * ratio;
			_odometer_acc += (uint32_t)rpm * dt;
			while(_odometer_acc >= step)
			{
				_odometer_acc -= step;
				odometer++;
			}
			
			return;
		}
		
		void _ReverseArray(uint8_t *array, uint8_t length)
		{
			uint8_t i = 0;
			uint8_t j = length - 1;
			uint8_t temp;
			while(i < j)
			{
				temp = array[i];
				array[i] = array[j];
				array[j] = temp;
				
				i++;
				j--;
			}

			return;
		}
		
		bool _initiated;				// Флаг того, что объект был добавлен в менеджер
		MotorManager *_manager;			// Указатель на менеджер
		uint8_t _idx;					// Индекс порта
		uint32_t _wheel_length;			// Длина окружности колеса, 0.1 мм (см. SetWheelDiameter)
		uint32_t _odometer_step;		// 100 м пути в об/мин x мс (без ratio)
		uint32_t _odometer_time;		// Время предыдущего пакета с оборотами, мс
		uint32_t _odometer_acc;			// Остаток пути меньше 100 м, об/мин x ratio x мс
		
		struct error_t
		{
			volatile error_code_t code = ERROR_NONE;
			error_code_t prev = ERROR_NONE;
		} _error;
	
	private:
		
};
