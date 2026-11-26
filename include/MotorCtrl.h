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

	#warning Rename Lock to Ignition

	
	// Управление передачей
	void SetGear(uint8_t idx, uint8_t gear)
	{
		shift_mask_gear_t bit;
		switch(gear)
		{
			case MotorData::GEAR_NEUTRAL: { bit = GEAR_NEUTRAL; break; }
			case MotorData::GEAR_FORWARD: { bit = GEAR_FORWARD_HI; break; }
			case MotorData::GEAR_REVERSE: { bit = GEAR_REVERSE; break; }
			case MotorData::GEAR_LOW: { bit = GEAR_FORWARD_LOW; break; }
			//case MotorData::GEAR_BOOST:   { bit = GEAR_BOOST; break; }
			default: { bit = GEAR_NEUTRAL; break; }
		}
		SPI::hc595.WriteByMask(idx, bit, GEAR_MASK);

		return;
	}
	
	// Запрос передачи
	uint8_t GetGear(uint8_t idx)
	{
		return Motors::manager.motor_data[idx].data.gear;
	}
	
	
	// Управление рекуперацией (тормозом)
	void SetBreak(uint8_t idx, uint8_t state)
	{
		SPI::hc595.SetState(idx, BREAK_RECOVERY_BIT, ((state == 0) ? false : true));
		
		return;
	}

	// Запрос состояния рекуперации (тормоза)
	uint8_t GetBreak(uint8_t idx)
	{
		bool state = SPI::hc595.GetState(idx, BREAK_RECOVERY_BIT);
		
		return ((state) ? 0xFF : 0x00);
	}


	// Управление питанием контроллера (замок зажигания)
	void SetLock(uint8_t idx, uint8_t state)
	{
		SPI::hc595.SetState(idx, LOCK_BIT, ((state == 0) ? false : true));
		
		return;
	}

	// Запрос состояния питания контроллера (замок зажигания)
	uint8_t GetLock(uint8_t idx)
	{
		bool state = SPI::hc595.GetState(idx, LOCK_BIT);
		
		return ((state) ? 0xFF : 0x00);
	}


	// Управление газом
	void SetThrottle(uint8_t idx, uint16_t raw)
	{
		const auto &cfg = Config::obj.body.throttle;
		uint16_t val = map_clump<uint16_t>(raw, 0, 5000, cfg.pwm_min, cfg.pwm_max);
		
		switch(idx)
		{
			case 0: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, val); break;
			case 1: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, val); break;
			default: break;
		}
		
		return;
	}

	// Вход в аварийный режим
	void SetFailsafe(uint8_t idx)
	{
		switch(idx)
		{
			case 0: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Config::obj.body.throttle.pwm_min); break;
			case 1: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Config::obj.body.throttle.pwm_min); break;
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
