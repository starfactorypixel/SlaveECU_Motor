#pragma once
#include <inttypes.h>
#include "ConfigData.h"

//extern CRC_HandleTypeDef hcrc;

namespace Config
{
	static constexpr uint16_t EEPROM_OFFSET_MAIN = 0;
	static constexpr uint16_t DATA_SIZE = 256;
	static constexpr uint16_t DATA_H_SIZE = 9;
	static constexpr uint16_t DATA_PAGE_SIZE = 32;
	static constexpr uint16_t EEPROM_OFFSET_MIRROR = DATA_SIZE + EEPROM_OFFSET_MAIN;
	static constexpr uint32_t MIRROR_TIME_SYNC = 10 * 60 * 1000;
	
	static_assert(EEPROM_OFFSET_MAIN % DATA_PAGE_SIZE == 0, "EEPROM_OFFSET_MAIN must be a multiple of 32!");
	
	// Общая структура всего блока данных
	struct __attribute__((packed)) eeprom_t
	{
		// Версия формата заголовка, а так-же флаг наличия записи в блоке (если 0x00 или 0xFF, то считаем что блок не инициализирован)
		uint8_t verison = 0x02;
		
		// Счётчик записей в память
		uint32_t counter = 0x00000001;
		
		// Блок полезных данных
		eeprom_body_t body;
		
		// Заполнитель пустого места в объекте
		uint8_t _reserved[ (DATA_SIZE - DATA_H_SIZE - sizeof(eeprom_body_t)) ];
		
		// Контрольная сумма всей структуры
		uint32_t crc32;
	} obj;
	static_assert(sizeof(eeprom_t) == DATA_SIZE, "Structures should have the same size!");
	
	
};
