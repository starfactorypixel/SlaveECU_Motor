#pragma once
#include <inttypes.h>
#include <drivers/SPI_ZD25Q80B.h>
#include <drivers/SPI_CAT25080.h>
#include <drivers/SPI_HC595.h>
#include "SPIFast.h"

extern SPI_HandleTypeDef hspi2;

namespace SPI
{
	inline void SPI_Config(const SPIManagerInterface::spi_config_t &config)
	{
		if(hspi2.Init.BaudRatePrescaler == config.prescaler && hspi2.Init.FirstBit == config.first_bit) return;

		
		hspi2.Init.BaudRatePrescaler = config.prescaler;
		hspi2.Init.FirstBit = config.first_bit;
		HAL_SPI_Init(&hspi2);
	}

	inline void SPI_Write(uint8_t *data, uint16_t length)
	{
		//HAL_SPI_Transmit(&hspi2, data, length, 100);
		HAL_SPI_WriteFast(&hspi2, data, length, 100);
	}

	inline void SPI_Read(uint8_t *data, uint16_t length)
	{
		//HAL_SPI_Receive(&hspi2, data, length, 100);
		HAL_SPI_ReadFast(&hspi2, data, length, 100);
	}

	inline void SPI_WriteRead(uint8_t *tx_data, uint8_t *rx_data, uint16_t length)
	{
		//HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, length, 200);
		HAL_SPI_WriteReadFast(&hspi2, tx_data, rx_data, length, 200);
	}
	
	
	SPIManager<3> manager(SPI_Config, SPI_Write, SPI_Read, SPI_WriteRead);
	SPI_ZD25Q80B flash({GPIOB, GPIO_PIN_12}, SPI_BAUDRATEPRESCALER_2);
	SPI_CAT25080 eeprom({GPIOA, GPIO_PIN_8}, SPI_BAUDRATEPRESCALER_8);
	SPI_HC595<2> hc595({GPIOA, GPIO_PIN_8}, {GPIOB, GPIO_PIN_3}, {GPIOB, GPIO_PIN_2}, SPI_BAUDRATEPRESCALER_8);
	
	
	void Setup123()
	{
	/*
		flash.ErasePage(0UL);
		uint8_t nor_data[flash.NOR_PAGE_SIZE];
		memset(nor_data, 0x00, sizeof(nor_data));
		nor_data[0] = 0x55;
		nor_data[0] = 0xEE;
		nor_data[17] = 0x66;
		nor_data[35] = 0x77;
		nor_data[118] = 0x88;
		nor_data[254] = 0x99;
		flash.WritePage(0UL, nor_data);
	*/

	/*
		uint8_t ee_write[32] = {0x00};
		ee_write[0] = 0xAA;
		ee_write[1] = 0xAA;
		ee_write[12] = 0xAA;
		ee_write[18] = 0xAA;
		ee_write[23] = 0xAA;
		ee_write[31] = 0xAA;
		eeprom.WritePage(0U, ee_write);
	*/
	
	}

	void qweqwerrr()
	{
		uint8_t ee_data[eeprom.EEPROM_PAGE_SIZE];
		eeprom.ReadPage(0U, ee_data);
		
		DEBUG_LOG_ARRAY_HEX("ee1", ee_data, sizeof(ee_data));
		Logger.PrintNewLine();
	};

	void qwewqeq()
	{
		uint8_t nor_data[flash.NOR_PAGE_SIZE];
		memset(nor_data, 0x00, sizeof(nor_data));
		flash.ReadPage(0UL, nor_data);

		DEBUG_LOG_TOPIC("NOR", "Start read\n");
		DEBUG_LOG_ARRAY_HEX("NOR", nor_data, sizeof(nor_data));
		Logger.PrintNewLine();
	}








	inline void Setup()
	{
		manager.AddDevice(flash);
		manager.AddDevice(eeprom);
		manager.AddDevice(hc595);
		
		hc595.OutputEnable();
		
		uint8_t dev_id[3] = {0x00};
		flash.ReadDevID(dev_id);
		DEBUG_LOG_TOPIC("NOR", "manufacturer ID: 0x%02X, memory type: 0x%02X, memory density: 0x%02X\n", dev_id[0], dev_id[1], dev_id[2]);

		uint8_t unique_id[16] = {0x00};
		flash.ReadUniqueID(unique_id);
		DEBUG_LOG_ARRAY_HEX("NOR", unique_id, sizeof(unique_id));
		Logger.PrintNewLine();
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		manager.Tick(current_time);

		static uint32_t last = 0;
		if(current_time - last > 1500)
		{
			last= current_time;
			//qwewqeq();
			//qweqwerrr();
		}

		current_time = HAL_GetTick();
		
		return;
	}
}
