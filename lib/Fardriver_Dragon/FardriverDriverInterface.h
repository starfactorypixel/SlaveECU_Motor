#pragma once

#include <inttypes.h>
#include <string.h>

class FardriverDriverInterface
{
	public:
		
		/*
			Проверяет пакет на валидность.
				uint8_t *buffer - Сырой массив принятых данных.
				uint8_t length - Длина массива принятых данных.
				return - Длина данных, которые были распознаны, или -1 если пакет не подошёл по длине, 0 если по формату.
		*/
		virtual int8_t IsValidPacket(uint8_t *buffer, uint8_t length) = 0;
};
