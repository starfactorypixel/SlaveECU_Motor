#pragma once
#include <inttypes.h>
#include <CUtils.h>

extern TIM_HandleTypeDef htim3;

namespace MotorCtrl
{
	
	enum shift_mask_gear_t : uint8_t
	{
		GEAR_NEUTRAL =		0b00000000,
		GEAR_FORWARD_LOW =	0b01000000,
		GEAR_FORWARD_HI =	0b00100000,
		GEAR_REVERSE =		0b10000000,
		GEAR_MASK =			0b11100000
	};
	static uint8_t BREAK_RECOVERY_BIT = 2;
	static uint8_t LOCK_BIT = 1;

	
	// Управление передачей
	void SetGear(uint8_t idx, shift_mask_gear_t gear)
	{
		SPI::hc595.WriteByMask(idx, gear, GEAR_MASK);
		
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
	
	// Управление рекупирацией (тормозом)
	void SetBreak(uint8_t idx, uint8_t state)
	{
		SPI::hc595.SetState(idx, BREAK_RECOVERY_BIT, ((state == 0) ? false : true));
		
		return;
	}
	
	// Управление питанием контроллера (замок зажигания)
	void SetLock(uint8_t idx, uint8_t state)
	{
		SPI::hc595.SetState(idx, LOCK_BIT, ((state == 0) ? false : true));
		
		return;
	}

	// 
	void SetThrottle(uint8_t idx, uint16_t raw)
	{
		uint16_t val = 0;
		
		if(raw > 0)
		{
			auto *cfg = &Config::obj.body.throttle;
			val = map_clump<uint16_t>(raw, cfg->pedal_min, cfg->pedal_max, cfg->pwm_min, cfg->pwm_max);
		}

		switch(idx)
		{
			case 0: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, val); break;
			case 1: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, val); break;
			default: break;
		}
		
		return;
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
