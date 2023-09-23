#pragma once

#include <inttypes.h>
#include <string.h>
#include "FardriverDriverInterface.h"

class FardriverDriver_2023 : public FardriverDriverInterface
{
	static constexpr uint8_t packet_init_rx[] = {0x41, 0x54, 0x2B, 0x56, 0x45, 0x52, 0x53, 0x49, 0x4F, 0x4E};
	static constexpr uint8_t packet_init_tx[] = {};
	static constexpr uint8_t packet_request_tx[] = {0xAA, 0x13, 0xEC, 0x07, 0x01, 0xF1, 0xA2, 0x5D};
	
	public:
		
		virtual int8_t IsValidPacket(uint8_t *buffer, uint8_t length)
		{
			if(buffer == nullptr || length < 10) return -1;
			
			if(memcmp(buffer, packet_init_rx, sizeof(packet_init_rx)) == 0) return sizeof(packet_init_rx);
			
			if(buffer[0] == 0xAA && (buffer[1] & 0x80) > 0 && length >= 16) return 16;
			
			return 0;
		}
		
	private:
		
		/*
			Расчёт CRC16.
			Параметры: Poly: 0x8005, Init: 0x7F3C, RefIn: true, RefOut: true, XorOut: false
		*/
		uint16_t _GetCRC(uint8_t *buffer, uint8_t length)
		{
			uint16_t crc = 0x7F3C;
			
			for(uint8_t idx = 0; idx < length; ++idx)
			{
				crc ^= (uint16_t)buffer[idx];
				for(uint8_t i = 8; i != 0; --i)
				{
					if( (crc & 0x0001) != 0)
					{
						crc >>= 1;
						crc ^= 0xA001;
					}
					else
					{
						crc >>= 1;
					}
				}
			}
			
			return crc;
		}
		
		
};
