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

#define Spiral_Right_Out	1
#define Spiral_Right_In 	2
#define Spiral_Left_Out 	4
#define Spiral_Left_In	  8
#define First_Round       10
typedef enum{
	NormalSpot = 0,
	CleanSpot = 1,
	WallSpot = 2,
}SpotType;

void Spot_Mode(SpotType spottype);
void Spot_WithCell(SpotType spottype,float radian);
void Spot_GetTarget(uint8_t spiral_type,float radian,std::list<Point32_t> *target,int32_t x_off,int32_t y_off);
uint8_t Random_Dirt_Event(void);
int8_t Spot_HandleException(SpotType st);
#endif /*----Behaviors------*/





