/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team Dxsong
  * @version V0.0
  * @date    11-July-2011
  * @brief   System Initialize
  * @define a lot of IO function for a easier look
  ******************************************************************************
  * Initialize the System Clock.ADC converter.EXTI.Timer and USART3
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */

#ifndef __SPOT_H__
#define __SPOT_H__

#include <vector>

#include "mathematics.h"
#include "core_move.h"

#define SPIRAL_RIGHT_OUT  1
#define SPIRAL_RIGHT_IN  2
#define SPIRAL_LEFT_OUT  4
#define SPIRAL_LEFT_IN    8
#define First_Round       10

typedef enum {
	NO_SPOT = 0,
	NORMAL_SPOT = 1,
	CLEAN_SPOT = 2,
} SpotType;

class SpotMovement {

private:

	float spot_diameter_;//

	std::vector<Cell_t> targets_;

	std::vector<Cell_t>::iterator tp_;//target pointer

	std::vector<Cell_t>::iterator bp_;//bumper pointer

	Cell_t stop_point_;

	Cell_t begin_point_;

	SpotType st_;

	uint8_t spot_init_;

	uint8_t is_direct_change_;

	uint8_t spiral_type_;

	uint8_t is_spot_init_;

	uint8_t is_stuck_;

	uint8_t sout_od_cnt_;//spiral out obs detect count

	uint8_t sin_od_cnt_;//spiral in obs detect count

	uint8_t np_chg_;//next point change

	uint16_t spot_bumper_cnt_;

public:

/*
 * @author mengshige1988@qq.com
 * @brief SpotMovement instruction
 * @param diameter in meters
 * @return None
 */
	SpotMovement(float diameter);

	~SpotMovement();

/*
 * @author mengshige1988@qq.com
 * @brief init spot while ready to spot movement
 * @param None
 * @return void
 */

	void spotInit(float diameter = 1.0, Cell_t cur_point = {0, 0});

/*
 * @author mengshige1988@qq.com
 * @brief spot deinit
 * @param None
 * @return None
 */

	void spotDeinit();

	uint8_t isSpotInit()
	{ return spot_init_; }

	static SpotMovement *instance();

/*
 * @author mengshige1988@qq.com
 * @brief when obstical detcet spot set stop point
 * @param stp(stop point)
 * @return None
 */
	void setStopPoint(Cell_t *stp);

/*
 * @author mengshige1988@qq.com
 * @brief when detect obstacle(rcon, cliff, bumper) from cm_linear_move_to_point() change spiral type
 * and set stop point
 * @param None
 * return spiral type
 */
	uint8_t spotChgType();

/*
 * @author mengshige1988@qq.com
 * @brief generate target points for spot move
 * @param1 sp_type
 *		sp_type;SPIRAL_RIGHT_OUT,SPIRAL_LEFT_OUT,SPIRAL_RIGHT_OUT,SPIRAL_LEFT_IN.
 * @param2 diameter
 *			spiral diameters in meters
 * @param3 *target
 *			target list pointer
 * @param4 begin point
 * @return None
 */
	void genTargets(uint8_t spiral_type, float radian, std::vector<Cell_t> *target, Cell_t curpoint);

/*
 * @author mengshige1988@qq.com
 * @brief get next spot target
 * @param next target Point 's address
 * @return 1 found ,0 not found
 * */
	int8_t spotNextTarget(Point32_t *next_point);

/*
* @author mengshige1988@qq.com
* @brief get the first nearest point.
* @param1 reference point.
* @return None.
* */

	uint8_t getNearPoint(Cell_t ref_point);

	void setSpiralType(uint8_t spi_t)
	{ spiral_type_ = spi_t; }

	void setSpiralObsDetectCnt(uint8_t sout, uint8_t sin)
	{
		sout_od_cnt_ = sout;
		sin_od_cnt_ = sin;
	}

	uint8_t isDirectChange(void)
	{ return is_direct_change_; }

	void setDirectChange(void)
	{ is_direct_change_ = 1; }

	void resetDirectChange(void)
	{ is_direct_change_ = 0; }

	uint8_t isStuck(void)
	{ return is_stuck_; }

	void setStuck(void)
	{ is_stuck_ = 1; }

	void resetStuck(void)
	{ is_stuck_ = 0; }

	void setBeginPoint(Cell_t begin)
	{
		begin_point_.X = begin.X;
		begin_point_.Y = begin.Y;
	}

	SpotType getSpotType(void)
	{ return st_; }

	void setSpotType(SpotType st)
	{ st_ = st; }

	void resetSpotType(void)
	{ st_ = NO_SPOT; }

	void setNextPointChange(void)
	{np_chg_ = 1;}

	void resetNextPointChange(void)
	{np_chg_ = 0;}

	uint8_t isNextPointChange(void)
	{return np_chg_;}
};

/*
 * legacy function remote ?
 * */
uint8_t Random_Dirt_Event(void);

#endif /*__SPOT_H__*/





