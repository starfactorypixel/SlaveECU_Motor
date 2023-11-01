
#include <inttypes.h>
#include <string.h>

typedef struct
{
	uint8_t voltage;
	int16_t current;
	int16_t temperature;
	// и т.п.
} motor_data_t;

class FardriverDriverInterface
{
public:
	/// @brief Проверяет пакет на целостность (CRC), корректность, соответствие протоколу и т.п.
	/// @param buffer Указатель на буффер с входящим пакетом
	/// @param length Размер пакета в байтах
	/// @return 'true' — пакет корректный, соответствует протоколу,
	///         'false' — пакет не корректный
	virtual bool IsValidPacket(uint8_t *buffer, uint8_t length) = 0;

	/// @brief Обрабатывает пакет,
	///        заполняет структуру с информацией по мотору и драйверу данными из пакета,
	///        если надо, формирует данные на отправку
	/// @param buffer Указатель на буффер с пакетом
	/// @param length Размер пакета в байтах
	/// @param motor_data [OUT] Структура с данными для заполнения
	/// @param time Время вызова. Может быть полезно для периодической отправки запросов драйвером
	/// @param tx_data_length [OUT]  сколько байт надо отправить.
	/// @param tx_buffer_size Размер буфера отправки.
	/// @param tx_buffer Указатель на буффер для отправки
	/// @return 'true' — пакет корректный, данные загружены,
	///			'false' — пакет не корректный, проигнорирован
	virtual bool ProcessPacket(uint8_t *buffer, uint8_t length,
							   motor_data_t &motor_data, uint32_t time,
							   uint8_t &tx_data_length, uint8_t tx_buffer_size = 0, uint8_t *tx_buffer = nullptr) = 0;
};

/*
	Контроллер флудит rx пакетом и чтобы установить связь нужно ответить на него tx пакетом.
	Чтобы запросить данные с контроллера нужно отправить пакет tx и получить пачку (38) rx пакетов.
*/
class FardriverDriverOld : public FardriverDriverInterface
{
	const uint8_t motor_packet_init_rx[16] = {0x31, 0x38, 0x37, 0x38, 0x38, 0x36, 0x39, 0x32, 0x3D, 0x53, 0x53, 0x41, 0x50, 0x2B, 0x54, 0x41};

	// TODO: для примера, нужно внести правильный!
	const uint8_t motor_packet_request_tx[16] = {0x01, 0x02, 0x03};
	uint32_t _last_request_time = 0;

	enum _driver__state_t
	{
		DS_WAITING_FOR_INIT,
		DS_INITIALIZED,
	};

	_driver__state_t _driver_state = DS_WAITING_FOR_INIT;

public:
	virtual bool ProcessPacket(uint8_t *buffer, uint8_t length,
							   motor_data_t &motor_data, uint32_t time,
							   uint8_t &tx_data_length, uint8_t tx_buffer_size = 0, uint8_t *tx_buffer = nullptr) override
	{
		if (!IsValidPacket(buffer, length))
			return false;

		switch (_driver_state)
		{
		case DS_WAITING_FOR_INIT: // ждём пакет инициализации
			if (memcmp(motor_packet_init_rx, buffer, length) == 0)
			{
				// должны ответить на пакет инициализации
				if (_RequestPackets(time, tx_data_length, tx_buffer_size, tx_buffer))
				{
					_driver_state = DS_INITIALIZED;
					return true;
				}

				// не смогли сформировать запрос =(
				// поэтому состояние не меняем
				return false;
			}

		case DS_INITIALIZED: // пакет иницициализации был, дальше должны быть только пакеты данных
			// не пора ли ещё раз отправить запрос пакетов?
			_RequestPackets(time, tx_data_length, tx_buffer_size, tx_buffer);

			// парсим входящий пакет
			if (buffer[0] == 0xAA && (buffer[1] & 0x80) == 0)
			{
				// TODO: для примера. Индексы от балды
				if (buffer[1] == 0xAB)
					motor_data.voltage = buffer[11];

				return true;
			}
		}

		// по идее сюда попасть не должны никогда
		return false;
	}

	virtual bool IsValidPacket(uint8_t *buffer, uint8_t length) override
	{
		if (buffer == nullptr || length == 0)
			return false;

		if (memcmp(motor_packet_init_rx, buffer, length) == 0)
			return true;

		if (length >= 2 && buffer[0] == 0xAA && (buffer[1] & 0x80) == 0)
			return true;

		return false;
	}

private:
	/// @brief Формирует запрос пакетов с данными в указанный буффер
	/// @param time Текущее время
	/// @param tx_data_length [OUT]  сколько байт надо отправить.
	/// @param tx_buffer_size Размер буфера отправки.
	/// @param tx_buffer Указатель на буффер для отправки
	/// @return 'true' — если формирование запроса прошло успешно
	///			'false' — если не удалось сформировать запрос или не пришло время его отправки
	bool _RequestPackets(uint32_t time, uint8_t &tx_data_length, uint8_t tx_buffer_size, uint8_t *tx_buffer)
	{
		// TODO: период запросов от балды
		if (tx_buffer_size >= sizeof(motor_packet_request_tx) && tx_buffer != nullptr && time - _last_request_time >= 100)
		{
			tx_data_length = sizeof(motor_packet_request_tx);
			memcpy(tx_buffer, motor_packet_request_tx, tx_data_length);
			_last_request_time = time;
			return true;
		}

		tx_data_length = 0; // отправлять нечего
		return false;
	}
};

/*
	Контроллер флудит пачками пакетов (55) самостоятельно
*/
class FardriverDriverNew : public FardriverDriverInterface
{
public:
	virtual bool ProcessPacket(uint8_t *buffer, uint8_t length,
							   motor_data_t &motor_data, uint32_t time,
							   uint8_t &tx_data_length, uint8_t tx_buffer_size = 0, uint8_t *tx_buffer = nullptr) override
	{
		if (!IsValidPacket(buffer, length))
			return false;

		// TODO: для примера. Индексы от балды
		if (buffer[1] == 0x81)
			motor_data.voltage = buffer[11];

		return true;
	}

	virtual bool IsValidPacket(uint8_t *buffer, uint8_t length) override
	{
		if (buffer == nullptr || length == 0)
			return false;

		if (length >= 2 && buffer[0] == 0xAA && (buffer[1] & 0x7F) < 0x37)
			return true;

		return false;
	}

private:
	/*
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
				0xE2, 0xE8, 0xEE, 0xD6, 0xDC, 0xF4, 0xFA, 0x00};

		return id_map[id & 0x7F];
	}
	*/
};

class FardriverDriverVoid : public FardriverDriverInterface
{
public:
	virtual bool ProcessPacket(uint8_t *buffer, uint8_t length,
							   motor_data_t &motor_data, uint32_t time,
							   uint8_t &tx_data_length, uint8_t tx_buffer_size = 0, uint8_t *tx_buffer = nullptr) override
	{
		tx_data_length = 0;
		return false;
	}

	virtual bool IsValidPacket(uint8_t *buffer, uint8_t length) override
	{
		return true;
	}
};

typedef struct
{
	uint8_t motor_idx;							// индекс мотора
	uint8_t rx_buffer[32];						// Приёмный буфер
	uint8_t rx_data_size;						// Длина данных в буфере
	bool rx_ready = true;						// Флаг готовности приёма данных (true - Готов принимать по прерыванию, false - Готов парсится в цикле)
	uint8_t tx_buffer[32];						// Буффер для данных на отправку
	uint8_t tx_data_size;						// длина данных в буфере на отправку
	FardriverDriverInterface *driver = nullptr; // актуальный интерфейс драйвера мотора
	motor_data_t motor_data;					// распарсенные данные по мотору

} motor_t;

class FardriverManager
{
	static constexpr uint8_t _max_motors = 2;
	// static constexpr uint8_t _max_drivers = 6;

	// using event_data_callback_t = void (*)(const uint8_t motor_idx, motor_packet_raw_t *packet);
	// using event_error_callback_t = void (*)(const uint8_t motor_idx, const motor_error_t code);
	// using error_callback_t = void (*)(const uint8_t motor_idx, const uint8_t code);
	using tx_callback_t = void (*)(const uint8_t idx, const uint8_t *buffer, uint8_t const length);

public:
	FardriverManager(tx_callback_t tx_callback) : _tx_callback(tx_callback)
	{
		for (uint8_t i = 0; i < _max_motors; i++)
		{
			_motors[i].motor_idx = i;
			_motors[i].driver = nullptr;
			memset(_motors[i].rx_buffer, 0, sizeof(_motors[i].rx_buffer));
			_motors[i].rx_data_size = 0;
			_motors[i].rx_ready = true;
			memset(_motors[i].tx_buffer, 0, sizeof(_motors[i].tx_buffer));
			_motors[i].tx_data_size = 0;

			// инициализация "распарсенных" данных
			// тут по идее надо не обнулять, а устанавливать начальные параметры, которые будут отображаться в системе, пока нет данных
			memset(&_motors[i].motor_data, 0, sizeof(motor_data_t));
		}
		
		return;
	}

	~FardriverManager()
	{
		for (motor_t &motor : _motors)
		{
			if (motor.driver != nullptr)
				delete motor.driver;
		}
	}

	void RXEvent(uint8_t idx, uint8_t *buffer, uint8_t length)
	{
		if (idx >= _max_motors)
			return;
		if (_motors[idx].rx_ready == false)
			return;
		if (length > sizeof(_motors[idx].rx_buffer))
			return;

		_motors[idx].rx_ready = false;
		memcpy(_motors[idx].rx_buffer, buffer, length);
		_motors[idx].rx_data_size = length;

		return;
	}

	void Processing(uint32_t time)
	{
		if (time - _last_time < 1)
			return;

		for (motor_t &motor : _motors)
		{
			if (motor.rx_ready == false)
			{
				// определяемся, какой драйвер должен обрабатывать данные с мотора
				// в самом методе _SelectDriver() выбор производим только если ранее драйвер не был выбран
				// по сути, делаем это только один раз при старте системы
				// замена драйвера "на горячую" — это вряд ли, поэтому по ходу работы меняться драйвер не будет
				_SelectDriver(motor);

				// Разбор пакета и формирование ответа (если надо)
				memset(motor.tx_buffer, 0, sizeof(motor.tx_buffer));
				motor.tx_data_size = 0;
				motor.rx_ready = motor.driver->ProcessPacket(motor.rx_buffer, motor.rx_data_size,
															 motor.motor_data, time,
															 motor.tx_data_size, sizeof(motor.tx_buffer), motor.tx_buffer);

				if (motor.rx_ready == false)
				{
					// Не получилось обработать пакет
					// что тут делать - ХЗ
					// может этот блок и не нужен
				}

				if (motor.tx_data_size > 0)
				{
					// есть данные для отправки в движок, надо отправлять
					_tx_callback(motor.motor_idx, motor.tx_buffer, motor.tx_data_size);
				}
			}
		}

		_last_time = time;

		return;
	}

private:
	motor_t _motors[_max_motors];

	tx_callback_t _tx_callback = nullptr;

	uint32_t _last_time = 0;

	// определяемся, какой драйвер должен обрабатывать данные с мотора
	void _SelectDriver(motor_t &motor)
	{
		// если драйвер уже выбран, то нечего мучиться
		if (motor.driver != nullptr)
			return;

		// пробуем старый драйвер
		motor.driver = new FardriverDriverOld();
		if (motor.driver->IsValidPacket(motor.rx_buffer, motor.rx_data_size))
			return; // подошел

		// старый драйвер не подходит, пробуем новый
		delete motor.driver;
		motor.driver = new FardriverDriverNew();
		if (motor.driver->IsValidPacket(motor.rx_buffer, motor.rx_data_size))
			return; // подошел

		// новый тоже не подошел, используем драйвер-заглушку
		delete motor.driver;
		motor.driver = new FardriverDriverVoid();
	}
};
