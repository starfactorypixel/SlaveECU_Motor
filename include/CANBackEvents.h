#pragma once
#include <inttypes.h>

namespace CANLib
{
	enum backevent_type_t : uint8_t
	{
		EVENT_CURR_NONE,
		EVENT_CURR_LIMIT,
	};
	
	void SoftEventOutputs(backevent_type_t type, uint8_t port, uint16_t val);
};
