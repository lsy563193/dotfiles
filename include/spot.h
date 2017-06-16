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

#ifndef __Spot_H
#define __Spot_H
#include <list>
#include "mathematics.h"

#define SPIRAL_RIGHT_OUT	1
#define SPIRAL_RIGHT_IN 	2
#define SPIRAL_LEFT_OUT 	4
#define SPIRAL_LEFT_IN	  8
#define First_Round       10
typedef enum{
	NORMAL_SPOT = 0,
	CLEAN_SPOT = 1,
	WALL_SPOT = 2,
}SpotType;

void spot_with_cell(SpotType spot_t,float diameter);
void gen_spot_target(uint8_t spiral_type,float radian,std::list<Point32_t> *target,int32_t x_off,int32_t y_off);
uint8_t Random_Dirt_Event(void);
int8_t spot_handle_exception(SpotType st);
#endif /*----Behaviors------*/





