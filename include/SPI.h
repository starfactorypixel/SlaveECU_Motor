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
	
	SPIManager<2> manager(SPI_Config, SPI_Write, SPI_Read, SPI_WriteRead);
	SPI_ZD25Q80B flash({GPIOB, GPIO_PIN_12}, SPI_BAUDRATEPRESCALER_2);
	SPI_HC595<2> hc595({GPIOA, GPIO_PIN_8}, {GPIOB, GPIO_PIN_3}, {GPIOB, GPIO_PIN_2}, SPI_BAUDRATEPRESCALER_8);
	// hc595 не имеет CS, поэтому используется не используемый пин A8
	
	
	inline void Setup()
	{
		manager.AddDevice(flash);
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
		
		current_time = HAL_GetTick();
		return;
	}
}
