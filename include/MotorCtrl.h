#pragma once
#include <inttypes.h>
#include <EasyPinA.h>
#include <CUtils.h>

extern TIM_HandleTypeDef htim3;

namespace MotorCtrl
{
	
	enum shift_mask_gear_t : uint8_t
	{
		GEAR_NEUTRAL =		0b00000000,
		GEAR_FORWARD_LOW =	0b00000001,
		GEAR_FORWARD_HI =	0b00000010,
		GEAR_REVERSE =		0b00000100,
		GEAR_MASK =			0b00000111
	};
	static uint8_t BREAK_RECOVERY_BIT = 4;
	static uint8_t LOCK_BIT = 7;

	
	// Управление передачей
	void SetGear(uint8_t idx, shift_mask_gear_t gear)
	{
		SPI::hc595.WriteByMask(idx, gear, GEAR_MASK);
		
		return;
	}
	
	// Управление рекупирацией (тормозом)
	void SetBreak(uint8_t idx, bool state)
	{
		SPI::hc595.SetState(idx, BREAK_RECOVERY_BIT, state);
		
		return;
	}

	// Управление питанием контроллера (замок зажигания)
	void SetLock(uint8_t idx, bool state)
	{
		SPI::hc595.SetState(idx, LOCK_BIT, state);
		
		return;
	}
	
	void SetGear(uint8_t idx, uint8_t mask)
	{
		shift_mask_gear_t gear;
		
		switch(mask)
		{
			case MotorManagerData::GEAR_NEUTRAL: { gear = GEAR_NEUTRAL; break; }
			case MotorManagerData::GEAR_FORWARD: { gear = GEAR_FORWARD_HI; break; }
			case MotorManagerData::GEAR_REVERSE: { gear = GEAR_REVERSE; break; }
			case MotorManagerData::GEAR_LOW: { gear = GEAR_FORWARD_LOW; break; }
			//case MotorManagerData::GEAR_BOOST:   { gear = GEAR_BOOST; break; }
			default: { gear = GEAR_NEUTRAL; break; }
		}
		
		return SetGear(idx, gear);
	}

	
	void HardwareSetup()
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Config::obj.body.throttle.pwm_min);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Config::obj.body.throttle.pwm_min);
		
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		
		return;
	}
	
	inline void Setup()
	{
		HardwareSetup();

		CANLib::obj_throttle_value_1.RegisterFunctionSetRealtime
		(
			// Колбек realtime данных
			[](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				uint16_t throttle = (can_frame.data[1] | (can_frame.data[2] << 8));
				
				auto *cfg = &Config::obj.body.throttle;
				
				uint16_t val = map_clump<uint16_t>(throttle, cfg->pedal_min, cfg->pedal_max, cfg->pwm_min, cfg->pwm_max);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, val);
				
				return CAN_RESULT_IGNORE;
			}, 
			// Колбек нарушения логики приёма
			[](uint32_t time_has_passed_ms) -> void
			{
				DEBUG_LOG_TOPIC("ThrlVal", "motor: 1, ERROR\n");
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				
				// Защита отключена.
				CANLib::obj_throttle_value_1.ResetRealtimeErrorState();
			}, 
			// Настройки
			100, 0, true, 3
		);
		CANLib::obj_throttle_value_1.ResetRealtimeErrorState();

		CANLib::obj_throttle_value_2.RegisterFunctionSetRealtime
		(
			// Колбек realtime данных
			[](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				uint16_t throttle = (can_frame.data[1] | (can_frame.data[2] << 8));
				
				auto *cfg = &Config::obj.body.throttle;
				
				uint16_t val = map_clump<uint16_t>(throttle, cfg->pedal_min, cfg->pedal_max, cfg->pwm_min, cfg->pwm_max);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, val);
				
				return CAN_RESULT_IGNORE;
			}, 
			// Колбек нарушения логики приёма
			[](uint32_t time_has_passed_ms) -> void
			{
				DEBUG_LOG_TOPIC("ThrlVal", "motor: 2, ERROR\n");
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

				// Защита отключена.
				CANLib::obj_throttle_value_2.ResetRealtimeErrorState();
			}, 
			// Настройки
			100, 0, true, 3
		);
		CANLib::obj_throttle_value_2.ResetRealtimeErrorState();
		
		CANLib::obj_transmission_value_1.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			SetGear(0, can_frame.data[0]);

			can_frame.function_id = CAN_FUNC_EVENT_OK;
			return CAN_RESULT_CAN_FRAME;
		});
		CANLib::obj_transmission_value_2.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			SetGear(1, can_frame.data[0]);

			can_frame.function_id = CAN_FUNC_EVENT_OK;
			return CAN_RESULT_CAN_FRAME;
		});

		CANLib::obj_brakerecuperation_flag_1.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			bool state = (can_frame.data[0] == 0) ? false : true;
			SetBreak(0, state);
			
			can_frame.function_id = CAN_FUNC_EVENT_OK;
			return CAN_RESULT_CAN_FRAME;
		});
		CANLib::obj_brakerecuperation_flag_2.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			bool state = (can_frame.data[0] == 0) ? false : true;
			SetBreak(1, state);
			
			can_frame.function_id = CAN_FUNC_EVENT_OK;
			return CAN_RESULT_CAN_FRAME;
		});
		
		CANLib::obj_ignitionlock_flag_1.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			bool state = (can_frame.data[0] == 0) ? false : true;
			SetLock(0, state);
			
			can_frame.function_id = CAN_FUNC_EVENT_OK;
			return CAN_RESULT_CAN_FRAME;
		});
		CANLib::obj_ignitionlock_flag_2.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			bool state = (can_frame.data[0] == 0) ? false : true;
			SetLock(1, state);
			
			can_frame.function_id = CAN_FUNC_EVENT_OK;
			return CAN_RESULT_CAN_FRAME;
		});
		
		return;
	}

	/*
	uint16_t val1 = 0;
	const uint32_t gist1 = 10;
	const uint32_t gist2 = 50;
	const uint32_t gist3 = 100;
	const uint32_t sample_counter = 5;
	uint32_t counter_pos = 0;
	uint32_t counter_neg = 0;
	uint32_t adc_val = 0;
	*/
	
	inline void Loop(uint32_t &current_time)
	{

/*
		static uint32_t tick_20 = 0;
		if(current_time - tick_20 > 20)
		{
			tick_20 = current_time;

		
			uint16_t throttle_adc = throttle.Get();
			throttle_adc >>= 2;
			
			if( throttle_adc > adc_val )
			{
				counter_pos++;
				counter_neg = 0;
			}
			else if( throttle_adc < adc_val )
			{
				counter_neg++;
				counter_pos = 0;
			}

			if( throttle_adc > (adc_val + gist3) )
			{
				if( counter_pos >= sample_counter )
					adc_val += gist3;
			}
			else if( throttle_adc < (adc_val - gist3) )
			{
				if( counter_neg >= sample_counter )
					adc_val -= gist3;
			}

			if( throttle_adc > (adc_val + gist2) && throttle_adc <= (adc_val + gist3) )
			{
				if( counter_pos >= sample_counter )
					adc_val += gist2;
			}
			else if( throttle_adc < (adc_val - gist2) && throttle_adc >= (adc_val - gist3) )
			{
				if( counter_neg >= sample_counter )
					adc_val -= gist2;
			}

			if( throttle_adc > (adc_val + gist1) )
			{
				if( counter_pos >= sample_counter )
					adc_val++;
			}
			else if( throttle_adc < (adc_val - gist1) )
			{
				if( counter_neg >= sample_counter )
					adc_val--;
			}
		}
*/		
		
		
		current_time = HAL_GetTick();
		
		return;
	}
}
