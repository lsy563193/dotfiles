#ifndef __CORMOVE_H__
#define __CORMOVE_H__

#include "mathematics.h"
#include "debug.h"
#include <vector>
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

extern uint8_t lowBattery;

void CM_HeadToCourse(uint8_t Speed,int16_t Angle);

MapTouringType CM_MoveToPoint(Point32_t Target);
bool CM_Check_is_exploring();
int CM_Get_grid_index(float position_x, float position_y, uint32_t width, uint32_t height, float resolution, double origin_x, double origin_y);
uint8_t CM_MoveForward(void);

uint8_t CM_Touring(void);

int8_t CM_MoveToCell( int16_t x, int16_t y, uint8_t mode, uint8_t length, uint8_t step );

void CM_CorBack(uint16_t dist);

void CM_SetGoHome(uint8_t remote);
void CM_TouringCancel(void);
void CM_SetGyroOffset(int16_t offset);

void CM_SetHome(int32_t x, int32_t y);
//void CM_SetStationHome(void);

void CM_ResetBoundaryBlocks(void);

void CM_AddTargets(Point16_t zone);
uint8_t CM_IsLowBattery(void);

uint8_t CM_CheckLoopBack(Point16_t target);

void CM_Matrix_Rotate(int32_t x_in, int32_t y_in, int32_t *x_out, int32_t *y_out, double theta);

MapTouringType CM_handleExtEvent(void);

#endif

