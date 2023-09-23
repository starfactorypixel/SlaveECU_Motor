#pragma once

#include <inttypes.h>
#include <string.h>
#include "FardriverDriverInterface.h"

class FardriverDriver_2022 : public FardriverDriverInterface
{
	// 0x31, 0x38, 0x37, 0x38, 0x38, 0x36, 0x39, 0x32, 0x3D, 0x53, 0x53, 0x41, 0x50, 0x2B, 0x54, 0x41 // обратный порядок
	static constexpr uint8_t packet_init_rx[] = {0x41, 0x54, 0x2B, 0x50, 0x41, 0x53, 0x53, 0x3D, 0x32, 0x39, 0x36, 0x38, 0x38, 0x37, 0x38, 0x31};
	static constexpr uint8_t packet_init_tx[] = {0x2B, 0x50, 0x41, 0x53, 0x53, 0x3D, 0x4F, 0x4E, 0x4E, 0x44, 0x4F, 0x4E, 0x4B, 0x45};
	static constexpr uint8_t packet_request_tx[] = {0xAA, 0x13, 0xEC, 0x07, 0x09, 0x6F, 0x28, 0xD7};
	
	public:
		
		virtual int8_t IsValidPacket(uint8_t *buffer, uint8_t length)
		{
			if(buffer == nullptr || length < 16) return -1;
			
			if(memcmp(buffer, packet_init_rx, sizeof(packet_init_rx)) == 0) return sizeof(packet_init_rx);
			
			if(buffer[0] == 0xAA && (buffer[1] & 0x80) == 0 && length >= 16) return 16;
			
			return 0;
		}
		
	private:
		
		/*
			Расчёт CRC16.
			Параметры: Алгебраическая сумма байт

			! Буфер данных должен быть инверсный !
		*/
		uint16_t _GetCRC(uint8_t *buffer, uint8_t length)
		{
			uint16_t crc = 0x0000;
			
			for(uint8_t idx = 2; idx < (length + 2); ++idx)
			{
				crc += buffer[idx];
			}
			
			return crc;
		}
};
