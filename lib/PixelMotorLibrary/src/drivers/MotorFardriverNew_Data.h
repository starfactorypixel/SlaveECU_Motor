#pragma once
#include <inttypes.h>

/*
	Fardriver, новый протокол.

	Контроллер сам шлёт по кругу 55 пакетов по 16 байт: AA | A1 | D0..D11 | CRC_lo CRC_hi.
	A1 = 0x80 | idx (0x80..0xB6). D0..D11 - шесть 16-битных little-endian слов «памяти» контроллера.
	Оперативные блоки повторяются 8 раз за цикл (~1.55 с):
		0x80,87,8E,95,9C,A3,AA,B0 - режим, ошибки, обороты, токи d/q;
		0x81,88,8F,96,9D,A4,AB,B1 - напряжение, ток батареи, газ;
		0x82,89,90,97,9E,A5,AC,B2 - фазные токи.
	Остальные пакеты - по одному разу за цикл (параметры, температуры, пробег).
*/
namespace FardriverNew
{
	// Пакет которым флудит контроллер. Запрос на подключение
	static constexpr uint8_t PacketInitRx[] = {0x41, 0x54, 0x2B, 0x56, 0x45, 0x52, 0x53, 0x49, 0x4F, 0x4E};

	// Пакет, которым нужно ответить, чтобы контроллер установил связь
	// AA cmd ~cmd p1 p2 p3 sum(0..5) ~sum
	static constexpr uint8_t PacketInitTx[] = {0xAA, 0x13, 0xEC, 0x07, 0x01, 0xF1, 0xA2, 0x5D};

	// Размер пакета принимаемых данных
	static constexpr uint8_t PacketSize = 16U;

	// Заголовок пакета
	static constexpr uint8_t PacketHeader[] = {0xAA};

	// Пакет запроса данных с контроллера (keep-alive, тот же что и PacketInitTx)
	static constexpr uint8_t PacketRequest[] = {0xAA, 0x13, 0xEC, 0x07, 0x01, 0xF1, 0xA2, 0x5D};

	// Интервал запроса данных из контроллера
	static constexpr uint16_t PacketRequestInerval = 1250;

	// Множитель оборотов: контроллер отдаёт об/мин * RpmRatio (настройка контроллера, по умолчанию 4)
	static constexpr uint8_t RpmRatio = 4U;

	// Биты error_flags, которые означают аварию. ПО Fardriver считает «есть ошибка» по (errors & 0x7FFF):
	// бит 15 в признак аварии не входит (предположительно - состояние тормоза).
	static constexpr uint16_t ErrorAlarmMask = 0x7FFF;


	// Передача, D0 биты 0-1 пакета 0x80
	enum gear_raw_t : uint8_t
	{
		GEAR_RAW_NEUTRAL = 0x00,
		GEAR_RAW_FORWARD = 0x01,
		GEAR_RAW_REVERSE = 0x02,
		GEAR_RAW_LOW = 0x03
	};

	// Типы ошибок контроллера (названия - из ПО Fardriver)
	typedef enum : uint16_t
	{
		MOTOR_ENCODER_ALARM = (1U << 0),		// 1. Датчики Холла
		ACCELERATION_PEDAL_ALARM = (1U << 1),	// 2. Газ (в т.ч. нажат при включении)
		CURRENT_PROTECT_RESTART = (1U << 2),
		PHASE_CURRENT_MUTATION = (1U << 3),
		VOLTAGE_ALARM = (1U << 4),				// Over / Under: уточняет packet_B3_t::over_voltage
		BURGLAR_ALARM = (1U << 5),
		MOTOR_OVER_TEMP = (1U << 6),
		CONTROLLER_OVER_TEMP = (1U << 7),
		PHASE_CURRENT_OVERFLOW = (1U << 8),
		PHASE_ZERO_ALARM = (1U << 9),
		PHASE_SHORT_ALARM = (1U << 10),			// Short / Lost: уточняет packet_B3_t::phase_lost
		LINE_CURR_ZERO_ALARM = (1U << 11),
		MOSFET_HIGHSIDE_BRIDGE_ALARM = (1U << 12),
		MOSFET_LOWSIDE_BRIDGE_ALARM = (1U << 13),
		MOE_CURRENT_ALARM = (1U << 14),
		BRAKE_ALARM = (1U << 15)				// Не авария (см. ErrorAlarmMask), предположительно тормоз
	} error_t;

	//
	struct __attribute__((__packed__)) packet_raw_t
	{
		uint8_t _A0;
		uint8_t _A1;
		uint8_t _D0;
		uint8_t _D1;
		uint8_t _D2;
		uint8_t _D3;
		uint8_t _D4;
		uint8_t _D5;
		uint8_t _D6;
		uint8_t _D7;
		uint8_t _D8;
		uint8_t _D9;
		uint8_t _D10;
		uint8_t _D11;
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_raw_t) == PacketSize, "Structures should have the same size!");

	// Блок 0xE2: 0x80, 0x87, 0x8E, 0x95, 0x9C, 0xA3, 0xAA, 0xB0
	struct __attribute__((__packed__)) packet_80_t
	{
		uint8_t _A0;
		uint8_t _A1;
		uint8_t gear : 2;			// D0 b0-1: gear_raw_t
		uint8_t speed_level : 2;	// D0 b2-3: переключатель скорости
		uint8_t against : 1;		// D0 b4: колесо вращается ПРОТИВ направления передачи (R - назад, остальные - вперёд)
		uint8_t moving : 1;			// D0 b5: 0 - стоит, 1 - вращается
		uint8_t weak : 1;			// D0 b6: 0 - MTPA, 1 - ослабление поля WEAK
		uint8_t _D0b7 : 1;			// D0 b7: в дампах всегда 1, ПО не использует
		uint8_t hall : 3;			// D1 b0-2: датчики Холла (000 и 111 - обрыв)
		uint8_t _D1b3 : 1;			// D1 b3: в дампах всегда 1 (ПО пишет «Pass Error», если 0)
		uint8_t _D1b4 : 4;
		error_t error_flags;		// D2-3, bitmask
		uint16_t modulation;		// D4-5, x1/128
		uint16_t rpm;				// D6-7, об/мин * RpmRatio
		int16_t current_d;			// D8-9, x1/16 A (предположительно Id)
		int16_t current_q;			// D10-11, x1/16 A (предположительно Iq)
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_80_t) == PacketSize, "Structures should have the same size!");

	// Блок 0xE8: 0x81, 0x88, 0x8F, 0x96, 0x9D, 0xA4, 0xAB, 0xB1
	struct __attribute__((__packed__)) packet_81_t
	{
		uint8_t _A0;
		uint8_t _A1;
		uint16_t voltage;			// D0-1, x0.1 V
		uint16_t voltage2;			// D2-3, x0.1 V (второе измерение, отличается на десятые)
		int16_t current;			// D4-5, x0.25 A, < 0 - рекуперация
		int16_t _D6;				// D6-7, x1/16 A, назначение не известно
		int16_t _D8;				// D8-9, x1/16 A, назначение не известно
		int16_t throttle;			// D10-11, x1
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_81_t) == PacketSize, "Structures should have the same size!");

	// Пакет 0x94 (адрес 0x69)
	struct __attribute__((__packed__)) packet_94_t
	{
		uint8_t _A0;
		uint8_t _A1;
		uint8_t _D0[8];
		uint16_t odometer_low;		// D8-9, младшее слово пробега
		uint8_t _D10;
		uint8_t _D11;
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_94_t) == PacketSize, "Structures should have the same size!");

	// Пакет 0x98 (адрес 0x7C)
	struct __attribute__((__packed__)) packet_98_t
	{
		uint8_t _A0;
		uint8_t _A1;
		uint8_t _D0[10];
		uint16_t odometer_high;		// D10-11, старшее слово пробега (вместе с 0x94: x0.1 km)
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_98_t) == PacketSize, "Structures should have the same size!");
	
	// Пакет 0x99 (адрес 0x82)
	struct __attribute__((__packed__)) packet_99_t
	{
		uint8_t _A0;
		uint8_t _A1;
		uint16_t throttle_voltage;	// D0-1, x0.01 V - напряжение на входе педали газа
		uint8_t _D2[10];
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_99_t) == PacketSize, "Structures should have the same size!");
	
	// Пакет 0xB3 (адрес 0xD6)
	struct __attribute__((__packed__)) packet_B3_t
	{
		uint8_t _A0;
		uint8_t _A1;
		uint8_t _D0;
		uint8_t _D1;
		uint16_t _D2b0 : 11;
		uint16_t phase_lost : 1;	// D2-3 b11: при PHASE_SHORT_ALARM 1 - обрыв фазы, 0 - КЗ
		uint16_t _D2b12 : 4;
		uint16_t _D4b0 : 15;
		uint16_t over_voltage : 1;	// D4-5 b15: при VOLTAGE_ALARM 1 - перенапряжение, 0 - недонапряжение
		uint8_t _D6;
		uint8_t _D7;
		uint8_t _D8;
		uint8_t _D9;
		int16_t temp_controller;	// D10-11, x1 °C
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_B3_t) == PacketSize, "Structures should have the same size!");

	// Пакет 0xB5 (адрес 0xF4)
	struct __attribute__((__packed__)) packet_B5_t
	{
		uint8_t _A0;
		uint8_t _A1;
		int16_t temp_motor;			// D0-1, x1 °C
		uint8_t _D2;
		uint8_t _D3;
		uint8_t _D4;
		uint8_t _D5;
		uint8_t _D6;
		uint8_t _D7;
		uint8_t _D8;
		uint8_t _D9;
		uint8_t _D10;
		uint8_t _D11;
		uint16_t _CRC;
	};
	static_assert(sizeof(packet_B5_t) == PacketSize, "Structures should have the same size!");

}
