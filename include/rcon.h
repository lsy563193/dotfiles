//
// Created by lsy563193 on 11/17/17.
//

#include <cstdint>
#ifndef PP_RCON_H
#define PP_RCON_H

#define RconBL_HomeL		(uint32_t)0x40000000
#define RconBL_HomeT		(uint32_t)0x20000000
#define RconBL_HomeR		(uint32_t)0x10000000

#define RconL_HomeL			(uint32_t)0x04000000
#define RconL_HomeT			(uint32_t)0x02000000
#define RconL_HomeR			(uint32_t)0x01000000

#define RconFL2_HomeL		(uint32_t)0x00400000
#define RconFL2_HomeT		(uint32_t)0x00200000
#define RconFL2_HomeR		(uint32_t)0x00100000

#define RconFL_HomeL		(uint32_t)0x00040000
#define RconFL_HomeT		(uint32_t)0x00020000
#define RconFL_HomeR		(uint32_t)0x00010000

#define RconFR_HomeL		(uint32_t)0x00004000
#define RconFR_HomeT		(uint32_t)0x00002000
#define RconFR_HomeR		(uint32_t)0x00001000

#define RconFR2_HomeL		(uint32_t)0x00000400
#define RconFR2_HomeT		(uint32_t)0x00000200
#define RconFR2_HomeR		(uint32_t)0x00000100

#define RconR_HomeL			(uint32_t)0x00000040
#define RconR_HomeT			(uint32_t)0x00000020
#define RconR_HomeR			(uint32_t)0x00000010

#define RconBR_HomeL		(uint32_t)0x00000004
#define RconBR_HomeT		(uint32_t)0x00000002
#define RconBR_HomeR		(uint32_t)0x00000001

#define RconAll_Home_T			(uint32_t)0x22222222
#define RconAll_Home_LR			(uint32_t)0x55555555
#define RconAll_Home_TLR		(uint32_t)0x77777777
#define RconFrontAll_Home_T		(uint32_t)0x02222220
#define RconFrontAll_Home_LR	(uint32_t)0x05555550
#define RconFrontAll_Home_TLR	(uint32_t)0x07777770
#define RconFront_Home_T		(uint32_t)0x00222200
#define RconFront_Home_LR		(uint32_t)0x00555500
#define RconFront_Home_TLR		(uint32_t)0x00777700
#define RconAll_R_HomeT			(uint32_t)0x00002222
#define RconAll_L_HomeT			(uint32_t)0x22220000

#ifdef VIRTUAL_WALL

#define RconBL_Wall          	(uint16_t)0x8000
#define RconL_Wall           	(uint16_t)0x4000
#define RconFL2_Wall           	(uint16_t)0x2000
#define RconFL_Wall           	(uint16_t)0x1000
#define RconFR_Wall           	(uint16_t)0x0800
#define RconFR2_Wall           	(uint16_t)0x0400
#define RconR_Wall           	(uint16_t)0x0200
#define RconBR_Wall           	(uint16_t)0x0100
#define RconBL_Wall_T           (uint16_t)0x0080
#define RconL_Wall_T           	(uint16_t)0x0040
#define RconFL2_Wall_T          (uint16_t)0x0020
#define RconFL_Wall_T           (uint16_t)0x0010
#define RconFR_Wall_T           (uint16_t)0x0008
#define RconFR2_Wall_T          (uint16_t)0x0004
#define RconR_Wall_T           	(uint16_t)0x0002
#define RconBR_Wall_T           (uint16_t)0x0001

#else

#define RconFL_Wall          	(uint16_t)0x0000
#define RconFL_Wall_T         (uint16_t)0x0000
#define RconFR_Wall          	(uint16_t)0x0000
#define RconFR_Wall_T         (uint16_t)0x0000
#define RconFL2_Wall         	(uint16_t)0x0000
#define RconFL2_Wall_T        (uint16_t)0x0000
#define RconFR2_Wall         	(uint16_t)0x0000
#define RconFR2_Wall_T        (uint16_t)0x0000
#define RconL_Wall           	(uint16_t)0x0000
#define RconL_Wall_T          (uint16_t)0x0000
#define RconR_Wall           	(uint16_t)0x0000
#define RconR_Wall_T          (uint16_t)0x0000
#define RconBL_Wall          	(uint16_t)0x0000
#define RconBL_Wall_T         (uint16_t)0x0000
#define RconBR_Wall          	(uint16_t)0x0000
#define RconBR_Wall_T         (uint16_t)0x0000
#endif// VIRTUAL_WALL

class Rcon {
public:
	void setStatus(uint32_t code) {
		rcon_status_ |= code;
	}

	void resetStatus(void) {
		rcon_status_ = 0;
	}

	uint32_t getStatus()
	{
		return rcon_status_;
	}

	uint32_t getAll(void);

	/*
	 * For checking if L/R/FL/FR/FL2/FR2 receive HomeT signal.
	 */
	uint32_t getForwardTop();

	uint32_t getWFRcon();

	uint32_t getNavRcon();

	enum {
		enum_start = 0,
		bl = 0,
		left = 1,
		fl2 = 2,
		fl = 3,
		fr = 4,
		fr2 = 5,
		right = 6,
		br = 7,
		enum_end = 7,
	};
	/*
	 * Convert rcon status from uint32_t to an enum.
	 */
	static int convertToEnum(uint32_t rcon_status);
	/*
	 * Convert rcon status from enum to an uint32_t.
	 */
	static uint32_t convertFromEnum(int _enum);

private:
	uint32_t rcon_status_{};

};

extern Rcon c_rcon;
#endif //PP_RCON_H
