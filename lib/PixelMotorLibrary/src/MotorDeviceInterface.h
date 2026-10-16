#pragma once
#include <inttypes.h>
#include <math.h>

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
			ERROR_AUTH = -5,		// Пакет авторизации не соотвествует контроллеру
			ERROR_CTRL = 1,			// Получен код ошибки от контроллера
			ERROR_LOST = 2,			// Потеря связи с контроллером
		};
		
		
		MotorDeviceInterface() : _initiated(false), _manager(nullptr), _idx(0)
		{

		}
		
		virtual void Init() = 0;
		virtual void Tick(uint32_t &time) = 0;
		virtual void RawRx(const uint8_t *raw, const uint8_t length) = 0;
		
		void SetWheelDiameter(uint16_t value)
		{
			float wheel_lenght = M_PI * (float)value;
			_speed_coefficient = (wheel_lenght * 60.0F) + 0.5F;
			
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
		uint8_t _idx;					// Индекс порт
		uint32_t _speed_coefficient;	// Коэффициент для расчёта скорости из RPM
		
		struct error_t
		{
			volatile error_code_t code = ERROR_NONE;
			error_code_t prev = ERROR_NONE;
		} _error;
	
	private:
		
};
