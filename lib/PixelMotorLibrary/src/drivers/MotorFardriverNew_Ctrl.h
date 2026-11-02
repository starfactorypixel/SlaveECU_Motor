#pragma once
#include <inttypes.h>
#include "MotorFardriverNew_Data.h"

/*
	Команды управления контроллером Fardriver (новый протокол).

	motor1.SendCmd(FdNew::CMD_THROTTLE, 1100, 3900);
	motor1.SendCmd(FdNew::CMD_SAVE);

	Новая команда: значение в cmd_t, функция сборки пакета и строка в Commands[].
	Ответа на запись нет: принятое значение видно в потоке, в пакете с этим адресом.
*/

namespace FardriverNew
{
	static uint8_t CmdThrottle(uint8_t *buf, const uint16_t *p);
	static uint8_t CmdSave(uint8_t *buf, const uint16_t *p);
	
	// Собрать пакет(ы) команды в buf по параметрам p. Возвращает длину, 0 - параметры неверны.
	using cmd_build_t = uint8_t (*)(uint8_t *buf, const uint16_t *p);
	
	// Команды
	enum cmd_t : uint8_t
	{
		CMD_THROTTLE,		// p0 - Low, p1 - High, мВ
		CMD_SAVE,			// без параметров
	};
	
	struct cmd_desc_t
	{
		cmd_t cmd;				// Команда
		uint8_t args;			// Число параметров
		cmd_build_t build;		// Обработчик
	};
	
	static constexpr cmd_desc_t Commands[] =
	{
		{ CMD_THROTTLE, 2, CmdThrottle },
		{ CMD_SAVE,     0, CmdSave     },
	};
	
	

	
	// Пакет записи count слов (1..6) в память контроллера с адреса addr:
	// AA | 0xC4 + n | addr | addr | слова little-endian (n байт) | CRC_lo CRC_hi. Возвращает длину, 0 - ошибка.
	static uint8_t CmdBuild(uint8_t *buf, uint8_t addr, const uint16_t *words, uint8_t count)
	{
		if(count == 0 || count > 6) return 0;
		uint8_t length = count * 2;

		buf[0] = 0xAA;
		buf[1] = 0xC4 + length;
		buf[2] = addr;
		buf[3] = addr;
		for(uint8_t i = 0; i < count; ++i)
		{
			buf[4 + i * 2] = words[i] & 0xFF;
			buf[5 + i * 2] = words[i] >> 8;
		}

		uint16_t crc = CRC16(buf, 4 + length);
		buf[4 + length] = crc & 0xFF;
		buf[5 + length] = crc >> 8;

		return 6 + length;
	}
	
	
	// Уставки педали газа, мВ, шаг 50 мВ: слово 0x08, младший байт - Low, старший - High.
	// Текущие значения контроллер отдаёт в пакете 0x84: D4 - Low, D5 - High.
	static uint8_t CmdThrottle(uint8_t *buf, const uint16_t *p)
	{
		uint16_t low = p[0] / 50;
		uint16_t high = p[1] / 50;
		if(low >= high || high > 0xFF) return 0;

		uint16_t word = (high << 8) | low;

		return CmdBuild(buf, 0x08, &word, 1);
	}
	
	// Сохранить параметры - как в ПО Fardriver: дата/время в слова 0x22..0x24, затем 0x0488 в слово 0xA0.
	// Часов в МК нет, поэтому дата постоянная - 2000-01-01 00:00:00 (ПО только показывает её как время сохранения).
	static uint8_t CmdSave(uint8_t *buf, const uint16_t *p)
	{
		(void)p;

		// год | месяц << 8, день | час << 8, минута | секунда << 8
		static constexpr uint16_t time[3] = { (1 << 8) | 0, (0 << 8) | 1, 0x0000 };
		static constexpr uint16_t save = 0x0488;

		uint8_t length = CmdBuild(buf, 0x22, time, 3);
		length += CmdBuild(buf + length, 0xA0, &save, 1);

		return length;
	}
}
