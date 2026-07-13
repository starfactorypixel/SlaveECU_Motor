#pragma once
#include <inttypes.h>

class IBlockInfoSender
{
	public:
		virtual ~IBlockInfoSender() = default;

		enum error_flag_t : uint8_t
		{
			ERROR_1 = (1 << 0),
			ERROR_2 = (1 << 1),
			ERROR_3 = (1 << 2),
			ERROR_4 = (1 << 3),
			ERROR_5 = (1 << 4),
			ERROR_6 = (1 << 5),
			ERROR_7 = (1 << 6),
			ERROR_8 = (1 << 7)
		};
		
		virtual void SetErrorFlag(error_flag_t flag, bool state) = 0;
		virtual bool GetErrorFlag(error_flag_t flag) = 0;
		virtual void SendWakeupMsg(uint8_t reason) = 0;
		virtual void SendErrorMsg(uint8_t group, uint8_t code, uint16_t subcode) = 0;
};

namespace CANLib
{
	enum backevent_type_t : uint8_t
	{
		EVENT_CURR_NONE,
		EVENT_CURR_LIMIT,
	};
	
	void SoftEventOutputs(backevent_type_t type, uint8_t port, uint16_t val);
};
