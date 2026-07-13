#pragma once
#include <DrakePinD.hpp>
#include <CANLibrary.h>
#include "CAN/CanBlockInfo.hpp"
#include "CAN/CanMotorThrottle.hpp"
#include "CAN/CanMotorCtrl.hpp"
#include "CAN/CanMotorParam.hpp"
#include "CAN/CanMotorGearRollParam.hpp"
#include "CAN/CanStreamObj.hpp"

extern CAN_HandleTypeDef hcan;
extern bool HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length);

namespace CANLib
{
	static_assert(ENV_CAN_FIRST_ID == 0x0100 || ENV_CAN_FIRST_ID == 0x0130, "'ENV_CAN_FIRST_ID' must be 0x0100 or 0x0130 only!");
	
	static constexpr uint8_t CFG_CANObjectsCount = 32;
	static constexpr uint8_t CFG_CANFrameBufferSize = 16;
	static constexpr uint16_t CAN_BASE_ID = ENV_CAN_FIRST_ID;

	DrakePinD can_rs({GPIOA, GPIO_PIN_15}, DrakePin::OutputOpenDrain, DrakePin::High);















	



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
		data.voltage = Analog::VoltCalc.GetmV( Analog::GetMuxValue(Analog::PORT_VIN) );
		data.current = 0;
		data.temperature = INT8_MIN;

		return;
	}


	CANManager<32> can_manager(&HAL_CAN_Send, &HAL_GetTick, &OnInterruptCtrl);

	CanBlockInfo obj_block_info(CAN_BASE_ID+0, OnStaticInfoReq, OnDynamicInfoReq);
	CanMotorThrottle obj_throttle_value_1(CAN_BASE_ID+4, Motors::MOTOR_1, MotorCtrl::SetThrottle);
	CanMotorThrottle obj_throttle_value_2(CAN_BASE_ID+5, Motors::MOTOR_2, MotorCtrl::SetThrottle);
	CanMotorCtrl obj_transmission_value_1(CAN_BASE_ID+6, Motors::MOTOR_1, MotorCtrl::SetGear);
	CanMotorCtrl obj_transmission_value_2(CAN_BASE_ID+7, Motors::MOTOR_2, MotorCtrl::SetGear);
	CanMotorCtrl obj_brakerecuperation_flag_1(CAN_BASE_ID+8, Motors::MOTOR_1, MotorCtrl::SetBreak);
	CanMotorCtrl obj_brakerecuperation_flag_2(CAN_BASE_ID+9, Motors::MOTOR_2, MotorCtrl::SetBreak);
	CanMotorCtrl obj_ignitionlock_flag_1(CAN_BASE_ID+10, Motors::MOTOR_1, MotorCtrl::SetLock);
	CanMotorCtrl obj_ignitionlock_flag_2(CAN_BASE_ID+11, Motors::MOTOR_2, MotorCtrl::SetLock);
	//obj_controller_errors_1
	//obj_controller_errors_2

	// Т.к. нету проверки manager.common_data_ready[0] == true то даже пусыте данные будут считаться валидными.
	// Вариант сделать common_data ввиде класса с методами get set и возвращаеть через get ошибку если данных нет или устарели

	auto &data1 = Motors::manager.common_data[Motors::MOTOR_1];
	auto &data2 = Motors::manager.common_data[Motors::MOTOR_2];

	CanMotorParam<uint16_t> obj_rpm_1(CAN_BASE_ID+14, &data1.rpm, 250);
	CanMotorParam<uint16_t> obj_rpm_2(CAN_BASE_ID+15, &data2.rpm, 250);
	CanMotorParam<uint16_t> obj_speed_1(CAN_BASE_ID+16, &data1.speed, 250);
	CanMotorParam<uint16_t> obj_speed_2(CAN_BASE_ID+17, &data2.speed, 250);
	CanMotorParam<uint16_t> obj_voltage_1(CAN_BASE_ID+18, &data1.voltage, 500);
	CanMotorParam<uint16_t> obj_voltage_2(CAN_BASE_ID+19, &data2.voltage, 500);
	CanMotorParam<int16_t> obj_current_1(CAN_BASE_ID+20, &data1.current, 500);
	CanMotorParam<int16_t> obj_current_2(CAN_BASE_ID+21, &data2.current, 500);
	CanMotorParam<int16_t> obj_power_1(CAN_BASE_ID+22, &data1.power, 500);
	CanMotorParam<int16_t> obj_power_2(CAN_BASE_ID+23, &data2.power, 500);
	CanMotorGearRollParam obj_gear_roll_1(CAN_BASE_ID+24, &data1.gear, &data1.roll, 500);
	CanMotorGearRollParam obj_gear_roll_2(CAN_BASE_ID+25, &data2.gear, &data2.roll, 500);
	CanMotorParam<int16_t> obj_temperature_motor_1(CAN_BASE_ID+26, &data1.temp_motor, 1000);
	CanMotorParam<int16_t> obj_temperature_motor_2(CAN_BASE_ID+27, &data2.temp_motor, 1000);
	CanMotorParam<int16_t> obj_temperature_controller_1(CAN_BASE_ID+28, &data1.temp_controller, 1000);
	CanMotorParam<int16_t> obj_temperature_controller_2(CAN_BASE_ID+29, &data2.temp_controller, 1000);
	CanMotorParam<uint32_t> obj_odometer(CAN_BASE_ID+30, &data1.odometer, 5000);													// может оба одометра?
	CanStreamObj<int8_t> obj_temperature(CAN_BASE_ID+31, TempStream::temperatures, sizeofarray(TempStream::temperatures));






	template <typename T> 
	static inline uint8_t GetTempStatus(T t)
	{
		if(t <= 70)
			return CAN_FUNC_TIMER_NORMAL;
		if(t <= 120)
			return CAN_FUNC_TIMER_WARNING;
		
		return CAN_FUNC_TIMER_CRITICAL;
	}





	
	void CAN_Enable()
	{
		HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_Start(&hcan);
		
		can_rs.Off();
		
		return;
	}
	
	void CAN_Disable()
	{
		HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_Stop(&hcan);
		
		can_rs.On();
		
		return;
	}
	
	inline void Setup()
	{
		can_rs.Init();


		obj_temperature_motor_1.SetValueClassifier(GetTempStatus);
		obj_temperature_motor_2.SetValueClassifier(GetTempStatus);
		obj_temperature_controller_1.SetValueClassifier(GetTempStatus);
		obj_temperature_controller_2.SetValueClassifier(GetTempStatus);
		
		can_manager.AddObject(obj_block_info);
		can_manager.AddObject(obj_throttle_value_1);
		can_manager.AddObject(obj_throttle_value_2);
		can_manager.AddObject(obj_transmission_value_1);
		can_manager.AddObject(obj_transmission_value_2);
		can_manager.AddObject(obj_brakerecuperation_flag_1);
		can_manager.AddObject(obj_brakerecuperation_flag_2);
		can_manager.AddObject(obj_ignitionlock_flag_1);
		can_manager.AddObject(obj_ignitionlock_flag_2);
		//can_manager.AddObject(obj_controller_errors_1);
		//can_manager.AddObject(obj_controller_errors_2);
		can_manager.AddObject(obj_rpm_1);
		can_manager.AddObject(obj_rpm_2);
		can_manager.AddObject(obj_speed_1);
		can_manager.AddObject(obj_speed_2);
		can_manager.AddObject(obj_voltage_1);
		can_manager.AddObject(obj_voltage_2);
		can_manager.AddObject(obj_current_1);
		can_manager.AddObject(obj_current_2);
		can_manager.AddObject(obj_power_1);
		can_manager.AddObject(obj_power_2);
		can_manager.AddObject(obj_gear_roll_1);
		can_manager.AddObject(obj_gear_roll_2);
		can_manager.AddObject(obj_temperature_motor_1);
		can_manager.AddObject(obj_temperature_motor_2);
		can_manager.AddObject(obj_temperature_controller_1);
		can_manager.AddObject(obj_temperature_controller_2);
		can_manager.AddObject(obj_odometer);
		can_manager.AddObject(obj_temperature);

		CAN_Enable();
		
		return;
	}

	inline void Loop(uint32_t &current_time)
	{
		can_manager.Processing(/*current_time*/);
		
		current_time = HAL_GetTick();
		return;
	}
}

IBlockInfoSender &BlockInfoSender = CANLib::obj_block_info;
