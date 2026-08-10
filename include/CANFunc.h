#pragma once
#include <inttypes.h>
#include <CUtils.h>

extern CAN_HandleTypeDef hcan;

namespace CANLib
{

	void OnInterruptCtrl(bool enable)
	{
		if(enable)
			__HAL_CAN_ENABLE_IT(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
		else
			__HAL_CAN_DISABLE_IT(&hcan, CAN_IT_TX_MAILBOX_EMPTY);
	}

	void OnStaticInfoReq(CanBlockInfo::block_info_static_t &data)
	{
		data.hw_ver = About::board_ver;
		data.hw_type = About::board_type;
		data.can_ver = About::can_ver;
		data.sw_ver = About::soft_ver;
		memcpy(data.sn, About::sn, sizeof(data.sn));
		memcpy(data.features, (const uint8_t *)&About::features, sizeof(data.features));

		return;
	}

	void OnDynamicInfoReq(CanBlockInfo::block_info_dynamic_t &data)
	{
		data.uptime = HAL_GetTick();
		data.voltage = Analog::GetInsideVoltage();
		data.current = Analog::GetInsideCurrent();
		data.temperature = Analog::GetInsideTemperature();

		return;
	}

	static const CanBlockCfg::config_item_t block_cfg_table[] =
	{
/*
		{1, sizeof(uint16_t), &Config::obj.body.in1.interval_ms},
		{2, sizeof(uint16_t), &Config::obj.body.in2.interval_ms},
		{3, sizeof(uint16_t), &Config::obj.body.in3.interval_ms},
		{4, sizeof(uint16_t), &Config::obj.body.in4.interval_ms},
		{5, sizeof(uint16_t), &Config::obj.body.in5.interval_ms},
		{6, sizeof(uint16_t), &Config::obj.body.in6.interval_ms},
*/
	};
	static constexpr uint8_t block_cfg_table_count = sizeofarray(block_cfg_table);
	
	void OnCfgSaveReset()
	{
		return;
	}
	
};