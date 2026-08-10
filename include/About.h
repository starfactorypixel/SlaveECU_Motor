#pragma once
#include <ConstantLibrary.h>
#include <CUtils_Crypto.h>

namespace About
{
	static constexpr char name[] = "MotorECU";
	static constexpr char desc[] = "Motor control board for Pixel project";
	static constexpr uint8_t board_type = Consts::BOARD_TYPE_MOTORS;	// 5 bits
	static constexpr uint8_t board_ver = 3;		// 3 bits
	static constexpr uint8_t soft_ver = 3;		// 6 bits
	static constexpr uint8_t can_ver = 1;		// 2 bits
	static constexpr char git[] = "https://github.com/starfactorypixel/SlaveECU_Motor";
	static uint8_t sn[8];
	static Consts::features_io_t features = {};
	
	inline void Setup()
	{
		GetSerialNumber64(sn);
		
		Logger.PrintNewLine();
		Logger.PrintTopic("INFO").Printf("%s, board:%d, soft:%d, can:%d\n", name, board_ver, soft_ver, can_ver);
		Logger.PrintTopic("INFO").Printf("Desc: %s\n", desc);
		Logger.PrintTopic("INFO").Printf("Build: %s %s\n", __DATE__, __TIME__);
		Logger.PrintTopic("INFO").Printf("SN: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n", sn[0], sn[1], sn[2], sn[3], sn[4], sn[5], sn[6], sn[7]);
		Logger.PrintTopic("INFO").Printf("GitHub: %s\n", git);
		Logger.PrintTopic("READY").PrintNewLine();
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		return;
	}
}
