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
#include "path_planning.h"


#define CLOCKWISE 1
#define ANTI_CLOCKWISE 2

#define CLOCKWISE_OUT  1
#define CLOCKWISE_IN  2
#define ANTI_CLOCKWISE_OUT  4
#define ANTI_CLOCKWISE_IN    8

#define First_Round       10

typedef enum {
	NO_SPOT = 0,
	NORMAL_SPOT = 1,
	CLEAN_SPOT = 2,
} SpotType;

class SpotMovement {

private:

	float spot_diameter_;//

	std::vector<Cell_t> *targets_;
	std::vector<Cell_t> targets_cw_;
	std::vector<Cell_t> targets_acw_;
	std::vector<Cell_t> targets_been_;
	std::list<Cell_t> target_last_;
	std::vector<Cell_t>::iterator tp_;//target pointer

	std::vector<Cell_t>::iterator bp_;//bumper pointer

	Cell_t near_cell_;

	Cell_t begin_cell_;

	SpotType st_;
	
	uint8_t go_last_point_;

	uint8_t spot_init_;

	uint8_t is_obs_trigger_;

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

	void spotInit(float diameter, Cell_t cur_point);

/*
 * @author mengshige1988@qq.com
 * @brief spot deinit
 * @param None
 * @return None
 */

	void spotDeinit();

	uint8_t isSpotInit()
	{ return spot_init_; }

	int8_t endSpot(PPTargetType *target_path);

	static SpotMovement *instance();

/*
 * @author mengshige1988@qq.com
 * @brief when obstical detcet spot set near cell
 * @param stp(stop point)
 * @return None
 */
	//uint8_t setNearCell(const Cell_t& cur_cell,Cell_t *stp);

/*
 * @author mengshige1988@qq.com
 * @brief generate target points
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
	uint8_t spotNextTarget(const Cell_t &cur_cell,PPTargetType *target);

/*
* @author mengshige1988@qq.com
* @brief get neighbour cell from current iterator.
* @param1 current interator.
* @param2 neighbour cell.
* @return None.
* */
	//void getNeighbourCell(std::vector<Cell_t>::iterator &bp,Cell_t *np);
/*
* @author mengshige1988@qq.com
* @brief get stright to the colume or row end.
* @param1 current target list interator
* @return None.
* */
	void stright2End(uint8_t spt,std::vector<Cell_t>::iterator &tp);

	/*
	* @author mengshige1988@qq.com
	* @brief put all the targets int target_path
	* @param1 target_path
	* @return None.
	* */
	void pushAllTargets(PPTargetType *target_path);

/*
 *
 * */
	void setSpiralType(uint8_t spi_t)
	{ spiral_type_ = spi_t; }

	void setSpiralObsDetectCnt(uint8_t sout, uint8_t sin)
	{
		sout_od_cnt_ = sout;
		sin_od_cnt_ = sin;
	}

	uint8_t isOBSTrigger(void)
	{ return is_obs_trigger_; }

	void setOBSTrigger(void)
	{ is_obs_trigger_ = 1; }

	void resetOBSTrigger(void)
	{ is_obs_trigger_ = 0; }

	uint8_t isStuck(void)
	{ return is_stuck_; }

	void setStuck(void)
	{ is_stuck_ = 1; }

	void resetStuck(void)
	{ is_stuck_ = 0; }

	void setBeginCell(Cell_t begin)
	{
		begin_cell_.X = begin.X;
		begin_cell_.Y = begin.Y;
	}

	SpotType getSpotType(void)
	{ return st_; }

	void setSpotType(SpotType st)
	{ st_ = st; }

	void resetSpotType(void)
	{ st_ = NO_SPOT; }

	void setNearCellChange(void)
	{np_chg_ = 1;}

	void resetNearCellChange(void)
	{np_chg_ = 0;}

	uint8_t isNearCellChange(void)
	{return np_chg_;}
};

/*
 * legacy function remote ?
 * */
uint8_t Random_Dirt_Event(void);

#endif /*__SPOT_H__*/





