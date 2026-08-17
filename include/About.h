#pragma once
#include <ConstantLibrary.h>
#include <CUtils_Crypto.h>

namespace About
{
	static constexpr const uint8_t board_type = Consts::BOARD_TYPE_MOTORS;
	static constexpr const uint8_t board_ver = 3;
	static constexpr const uint8_t soft_ver = 3;
	static constexpr const uint8_t can_ver = 1;
	
	using Board = Boards::Info<board_type>;
	static uint8_t sn[8];
	Board::features_t features = {};

	inline void Setup()
	{
		GetSerialNumber64(sn);

		Logger.PrintNewLine();
		Logger.PrintTopic("INFO").Printf("%s, board:%d, soft:%d, can:%d\n", Board::name, board_ver, soft_ver, can_ver);
		Logger.PrintTopic("INFO").Printf("Desc: %s\n", Board::desc);
		Logger.PrintTopic("INFO").Printf("Build: %s %s\n", __DATE__, __TIME__);
		Logger.PrintTopic("INFO").Printf("SN: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n", sn[0],sn[1],sn[2],sn[3],sn[4],sn[5],sn[6],sn[7]);
		Logger.PrintTopic("INFO").Printf("GitHub: %s\n", Board::git);

		// Please don't delete this. It's very important to me.
		Logger.PrintTopic("INFO").Printf("In memory of my beloved Grandfather. 22.03.1942 - 06.12.2023\n");

		Logger.PrintTopic("READY").PrintNewLine();

		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		return;
	}
}
