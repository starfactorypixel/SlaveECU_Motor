
#include <inttypes.h>
#include <string.h>

class FardriverDriverInterface
{
	public:
		
		virtual bool Parse(uint8_t *buffer, uint8_t length) = 0;
		virtual bool ToHere(uint8_t *buffer, uint8_t length) = 0;
};

/*
	Контроллер флудит rx пакетом и чтобы установить связь нужно ответить на него tx пакетом.
	Чтобы запросить данные с контроллера нужно отправить пакет tx и получить пачку (38) rx пакетов.
*/
class FardriverDriverOld : public FardriverDriverInterface
{
	const uint8_t motor_packet_init_rx[16] = {0x31, 0x38, 0x37, 0x38, 0x38, 0x36, 0x39, 0x32, 0x3D, 0x53, 0x53, 0x41, 0x50, 0x2B, 0x54, 0x41};
	
	public:

		virtual bool Parse(uint8_t *buffer, uint8_t length) override
		{

		}

		virtual bool ToHere(uint8_t *buffer, uint8_t length) override
		{
			if(buffer[0] == 0xAA && (buffer[1] & 0x80) == 0) return true;
			if(memcmp(motor_packet_init_rx, buffer, length) == 0) return true;
			
			return false;
		}
};


/*
	Контроллер флудит пачками пакеток (55) самостоятельно
*/
class FardriverDriverNew : public FardriverDriverInterface
{
	public:

		virtual bool Parse(uint8_t *buffer, uint8_t length) override
		{
			
		}
		
		virtual bool ToHere(uint8_t *buffer, uint8_t length) override
		{
			if(buffer[0] == 0xAA && (buffer[1] & 0x7F) < 0x37) return true;
			
			return false;
		}

	private:
		
		uint8_t GetRealFuncID(uint8_t id)
		{
			static uint8_t id_map[] = 
			{
				0xE2, 0xE8, 0xEE, 0x00, 0x06, 0x0C, 0x12, 0xE2, 
				0xE8, 0xEE, 0x18, 0x1E, 0x24, 0x2A, 0xE2, 0xE8, 
				0xEE, 0x30, 0x5D, 0x63, 0x69, 0xE2, 0xE8, 0xEE, 
				0x7C, 0x82, 0x88, 0x8E, 0xE2, 0xE8, 0xEE, 0x94, 
				0x9A, 0xA0, 0xA6, 0xE2, 0xE8, 0xEE, 0xAC, 0xB2, 
				0xB8, 0xBE, 0xE2, 0xE8, 0xEE, 0xC4, 0xCA, 0xD0, 
				0xE2, 0xE8, 0xEE, 0xD6, 0xDC, 0xF4, 0xFA, 0x00
			};
			
			return id_map[id & 0x7F];
		}

};

class FardriverDriverVoid : public FardriverDriverInterface
{
	public:

		virtual bool Parse(uint8_t *buffer, uint8_t length) override
		{
			return false;
		}

		virtual bool ToHere(uint8_t *buffer, uint8_t length) override
		{
			return true;
		}
};


class FardriverManager
{
	static constexpr uint8_t _max_motors = 2;
	static constexpr uint8_t _max_drivers = 6;

	//using event_data_callback_t = void (*)(const uint8_t motor_idx, motor_packet_raw_t *packet);
	//using event_error_callback_t = void (*)(const uint8_t motor_idx, const motor_error_t code);
	//using error_callback_t = void (*)(const uint8_t motor_idx, const uint8_t code);
	using tx_callback_t = void (*)(const uint8_t idx, const uint8_t *buffer, uint8_t const length);
	
	public:

		FardriverManager(tx_callback_t tx_callback) : _tx_callback(tx_callback)
		{
			return;
		}
		
		void RXEvent(uint8_t idx, uint8_t *buffer, uint8_t length)
		{
			if(idx >= _max_motors) return;
			if(_motors[idx].rx_ready == false) return;
			
			_motors[idx].rx_ready = false;
			memcpy(_motors[idx].rx_buffer, buffer, length);
			_motors[idx].rx_buffer_length = length;
			
			return;
		}
		
		void Processing(uint32_t time)
		{
			if(time - _last_time < 1) return;
			
			for(auto &motor : _motors)
			{
				if(motor.rx_ready == false)
				{
					uint8_t p_idx = (motor.driver_idx == 255) ? 0 : motor.driver_idx;
					for(; p_idx < _max_drivers; ++p_idx)
					{
						if( _drivers[p_idx]->Parse(motor.rx_buffer, motor.rx_buffer_length) == true )
						{
							// Разбор пакета
							// На этом этапе проблемы, пока нету идей как и куда складывать или передавать уже готовые данные.
							// Так-же драйвер может захотеть чтото отправить в ответ, это тожу нужно учесть.
							
							motor.driver_idx = p_idx;
							motor.rx_ready = true;
							
							break;
						}
					}
					
					if(motor.rx_ready == false)
					{
						// Не нашлось подходящего драйвера
					}
				}
			}
			
			_last_time = time;

			return;
		}

	private:

		FardriverDriverInterface *_drivers[_max_drivers] = 
		{
			new FardriverDriverOld(),
			new FardriverDriverNew(),
			new FardriverDriverVoid(),
			new FardriverDriverVoid(),
			new FardriverDriverVoid(),
			new FardriverDriverVoid()
		};

		struct
		{
			uint8_t rx_buffer[32];			// Приёмный буфер
			uint8_t rx_buffer_length;		// Длина данный в буфере
			bool rx_ready = true;			// Флаг готовности приёма данных (true - Готов принимать по прерыванию, false - Готов парсится в цикле)
			uint8_t driver_idx = 255;		// Индекса драйвера

		} _motors[_max_motors];


		tx_callback_t _tx_callback = nullptr;


		uint32_t _last_time = 0;


		

};
