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

#ifndef __WallFollowMulti_H
#define __WallFollowMulti_H

#include "config.h"

#include "mathematics.h"
#include "debug.h"

typedef enum {
	Map_Wall_Follow_None = 0,
	Map_Wall_Follow_Escape_Trapped,
} MapWallFollowType;

typedef enum {
	Map_Escape_Trapped_Escaped,
	Map_Escape_Trapped_Trapped,
	Map_Escape_Trapped_Timeout,
} MapEscapeTrappedType;

typedef struct {
	int32_t front_obs_val;
	int32_t left_bumper_val;
	int32_t right_bumper_val;
} MapWallFollowSetting;

uint8_t Map_Wall_Follow(MapWallFollowType follow_type);

#endif/*----Behaviors------*/
