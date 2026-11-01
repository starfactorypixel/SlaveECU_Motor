#pragma once
#include <inttypes.h>

namespace FardriverOld
{
	// Пакет которым флудит контроллер. Запрос на подключение
	static constexpr uint8_t PacketInitRx[] = {0x41, 0x54, 0x2B ,0x50 ,0x41 ,0x53, 0x53, 0x3D ,0x32, 0x39, 0x36, 0x38, 0x38, 0x37, 0x38, 0x31};
	
	// Пакет, которым нужно ответить, чтобы контроллер установил связь
	static constexpr uint8_t PacketInitTx[] = {0x2B, 0x50, 0x41, 0x53, 0x53, 0x3D, 0x4F, 0x4E, 0x4E, 0x44, 0x4F, 0x4E, 0x4B, 0x45};
	
	// Размер пакета принимаемых данных
	static constexpr uint8_t PacketSize = 16U;
	
	// Заголовок пакета
	static constexpr uint8_t PacketHeader[] = {0xAA};
	
	// Пакет запроса данных с контроллера
	static constexpr uint8_t PacketRequest[] = {0xAA, 0x13, 0xEC, 0x07, 0x09, 0x6F, 0x28, 0xD7};
	
	// Интервал запроса данных из контроллера, мс
	static constexpr uint16_t PacketRequestInterval = 550;
	
	// Через сколько мс без пакетов считать связь потерянной
	static constexpr uint16_t LinkTimeout = 1000;
	
	// Множитель оборотов: контроллер отдаёт об/мин * RpmRatio
	static constexpr uint8_t RpmRatio = 4U;
	
	
	// Типы ошибок контроллера
	typedef enum : uint16_t
	{
		MOTOR_ENCODER_ALARM = (1U << 0),
		ACCELERATION_PEDAL_ALARM = (1U << 1),
		CURRENT_PROTECT_RESTART = (1U << 2),
		PHASE_CURRENT_MUTATION = (1U << 3),
		VOLTAGE_ALARM = (1U << 4),
		BURGLAR_ALARM = (1U << 5),
		MOTOR_OVER_TEMP = (1U << 6),
		CONTROLLER_OVER_TEMP = (1U << 7),
		PHASE_CURRENT_OVERFLOW = (1U << 8),
		PHASE_ZERO_ALARM = (1U << 9),
		PHASE_SHORT_ALARM = (1U << 10),
		LINE_CURR_ZERO_ALARM = (1U << 11),
		MOSFET_HIGHSIDE_BRIDGE_ALARM = (1U << 12),
		MOSFET_LOWSIDE_BRIDGE_ALARM = (1U << 13),
		MOE_CURRENT_ALARM = (1U << 14),
		PARCING_ALARM = (1U << 15)
	} error_t;
	
	// 
	struct __attribute__((__packed__)) packet_raw_t
	{
		uint16_t _CRC;
		uint8_t _D11;
		uint8_t _D10;
		uint8_t _D9;
		uint8_t _D8;
		uint8_t _D7;
		uint8_t _D6;
		uint8_t _D5;
		uint8_t _D4;
		uint8_t _D3;
		uint8_t _D2;
		uint8_t _D1;
		uint8_t _D0;
		uint8_t _A1;
		uint8_t _A0;
	};
	static_assert(sizeof(packet_raw_t) == PacketSize, "Structures should have the same size!");
	
	// 
	struct __attribute__((__packed__)) packet_00_t
	{
		uint16_t _CRC;
		uint16_t idout;
		uint16_t iqout;
		error_t error_flags;
		uint16_t rpm;
		uint8_t follow;
		uint8_t gear : 4;			// б0-1 передача: 0 N, 1 D, 2 R, 3 Low; б2-3 уровень скорости
		uint8_t roll : 4;			// 0 - стоит, 1 - вращается в сторону передачи, 2 и больше - против
		uint8_t hall;
		uint8_t mtpa_angle;
		uint8_t _A1;
		uint8_t _A0;
	};
	static_assert(sizeof(packet_00_t) == PacketSize, "Structures should have the same size!");
	
	// 
	struct __attribute__((__packed__)) packet_01_t
	{
		uint16_t _CRC;
		uint16_t throttle;
		uint16_t idin;
		uint16_t iqin;
		uint8_t work_stat;
		uint8_t mod_ratio;
		int16_t current;
		uint16_t voltage;
		uint8_t _A1;
		uint8_t _A0;
	};
	static_assert(sizeof(packet_01_t) == PacketSize, "Structures should have the same size!");
	
	// 
	struct __attribute__((__packed__)) packet_04_t
	{
		uint16_t _CRC;
		uint8_t _D11;
		uint8_t _D10;
		uint8_t _D9;
		uint8_t _D8;
		uint8_t _D7;
		uint8_t _D6;
		uint8_t _D5;
		uint8_t _D4;
		uint8_t _D3;
		uint8_t temp_controller;
		uint8_t _D1;
		uint8_t _D0;
		uint8_t _A1;
		uint8_t _A0;
	};
	static_assert(sizeof(packet_04_t) == PacketSize, "Structures should have the same size!");
	
	// 
	struct __attribute__((__packed__)) packet_0D_t
	{
		uint16_t _CRC;
		uint8_t _D11;
		uint8_t _D10;
		uint8_t _D9;
		uint8_t _D8;
		uint8_t _D7;
		uint8_t _D6;
		uint8_t _D5;
		uint8_t _D4;
		uint8_t _D3;
		uint8_t _D2;
		uint8_t _D1;
		uint8_t temp_motor;
		uint8_t _A1;
		uint8_t _A0;
	};
	static_assert(sizeof(packet_0D_t) == PacketSize, "Structures should have the same size!");

}
