/**
  ******************************************************************************
  * @file    AI Cleaning Robot
  * @author  ILife Team
  * @version V0.0
  * @date    09-Aug-2016
  * @brief   System Initialize
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
  ******************************************************************************
  */  

#ifndef __ROUNDING_H
#define __ROUNDING_H

#include "map.h"

typedef enum {
	ROUNDING_LEFT = 0,
	ROUNDING_RIGHT,
} RoundingType;

uint8_t rounding(RoundingType type, Point32_t target);

#endif
