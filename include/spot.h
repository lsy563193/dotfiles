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

#include <list>

#include "mathematics.h"
#include "core_move.h"

#define SPIRAL_RIGHT_OUT	1
#define SPIRAL_RIGHT_IN 	2
#define SPIRAL_LEFT_OUT 	4
#define SPIRAL_LEFT_IN	  8
#define First_Round       10
#define OBS_DETECT_COUNT_MAX 3

typedef enum{
	NO_SPOT = 0,
	NORMAL_SPOT = 1,
	CLEAN_SPOT = 2,
}SpotType;

class SpotMovement;

extern uint8_t g_should_follow_wall;

class SpotMovement
{

private:

	float spot_diameter_;//

	std::list<Point32_t> target_;

	std::list<Point32_t>::const_iterator tp_;//target pointer

	Point32_t stop_point_;

	Point32_t near_point_;

	Point32_t begin_point_;	

	SpotType st_;

	uint8_t spot_init_;

	uint8_t is_direct_change_;

	uint8_t spiral_type_;
	
	uint8_t is_spot_init_;

	uint8_t is_stuck_;

	uint8_t sout_od_cnt_;//spiral out obs detect count

	uint8_t sin_od_cnt_;//spiral in obs detect count
	
public:

	SpotMovement(float diameter);
	~SpotMovement();

	void spotInit(float diameter=1.0, Point32_t cur_point = {0,0});

    void spotDeinit();

    uint8_t isSpotInit() { return spot_init_; }

	static SpotMovement* instance();

	//void spotWithTarget(SpotType spot_t,float diameter);

	uint8_t changeSpiralType();

	void generateTarget(uint8_t spiral_type, float radian, std::list<Point32_t> *target, Point32_t curpoint);

	int8_t getNextTarget(Point32_t &next_point);

	uint8_t findNearestPoint(Point32_t ref_point);

	void setSpiralType(uint8_t spi_t){ spiral_type_ = spi_t;}

	void setSpiralObsDetectCnt(uint8_t sout,uint8_t sin){ sout_od_cnt_ = sout; sin_od_cnt_ = sin;}

	uint8_t isDirectChange(void){ return is_direct_change_; }
	
	void setDirectChange(void){	is_direct_change_ = 1;	}

	void resetDirectChange(void){ is_direct_change_ = 0; }

	uint8_t isStuck(void){ return is_stuck_; }

	void setStuck(void){ is_stuck_ = 1; }

	void resetStuck(void){ is_stuck_ = 0; }

	void setBeginPoint(Point32_t begin){ begin_point_.X = begin.X; begin_point_.Y = begin.Y; }

	SpotType getSpotType(void) { return st_; }

	void setSpotType(SpotType st){ st_ = st; }

	void resetSpotType(void){st_ = NO_SPOT; }
};

/*---legacy function , remove??----*/
uint8_t Random_Dirt_Event(void);

#endif /*__SPOT_H__*/





