/**
  ******************************************************************************
  * @file    stm32f10x_exti.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the EXTI firmware
  *          library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CORMOVE_H
#define __CORMOVE_H

#include "mathematics.h"
#include "debug.h"

#define MS_Clear 		0x00
#define MS_OBS   		0x01
#define MS_Bumper		0x02
#define MS_Cliff 		0x04
#define MS_User 		0x08
#define MS_Home			0x10
#define MS_Clean 		0x20
#define MS_Spot 		0x40
#define MS_Error 		0x80

#define TILT_PITCH_LIMIT	(100)
#define TILT_ROLL_LIMIT		(100)

#define COR_BACK_20MM		(120)
#define COR_BACK_100MM		(600)
#define COR_BACK_500MM		(3000)

typedef enum {
	MT_None = 0,
	MT_Battery,
	MT_Remote_Home,
	MT_Remote_Clean,
	MT_Remote_Spot,
	MT_Cliff,
	MT_Key_Clean,
	MT_Battery_Home,
} MapTouringType;

typedef struct {
	Point16_t	pos;
} VWType;

void CM_HeadToCourse(uint8_t Speed,int16_t Angle);

MapTouringType CM_MoveToPoint(Point32_t Target);

uint8_t CM_MoveForward(void);

uint8_t CM_Touring(void);

int8_t CM_MoveToCell( int16_t x, int16_t y, uint8_t mode, uint8_t length, uint8_t step );

void CM_CorBack(uint16_t dist);

void CM_SetGoHome(uint8_t remote);
void CM_TouringCancel(void);
void CM_SetGyroOffset(int16_t offset);

void CM_SetHome(int32_t x, int32_t y);
void CM_SetStationHome(void);

void CM_ResetBoundaryBlocks(void);

void CM_AddTargets(Point16_t zone);
uint8_t CM_IsLowBattery(void);

uint8_t CM_CheckLoopBack(Point16_t target);

void CM_Matrix_Rotate(int32_t x_in, int32_t y_in, int32_t *x_out, int32_t *y_out, double theta);

MapTouringType CM_handleExtEvent(void);

#endif

