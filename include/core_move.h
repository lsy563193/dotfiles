#ifndef __CORMOVE_H__
#define __CORMOVE_H__

#include "mathematics.h"
#include "debug.h"
#include <vector>
#include <bitset>
#define MS_Clear 		0x00
#define MS_OBS   		0x01
#define MS_Bumper		0x02
#define MS_Cliff 		0x04
#define MS_User 		0x08
#define MS_Home			0x10
#define MS_Clean 		0x20 #define MS_Spot 		0x40
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
	MT_Bumper,
	MT_OBS,
	MT_Boundary,
	MT_CurveMove,
	MT_Key_Clean,
	MT_Battery_Home,
} MapTouringType;

typedef enum {
	ACTION_NONE	= 0x01,
	ACTION_GO	= 0x02,
	ACTION_BACK	= 0x04,
	ACTION_LT	= 0x08,
	ACTION_RT	= 0x10,
} ActionType;

typedef struct {
	Point16_t	pos;
} VWType;

extern uint8_t lowBattery;

void CM_HeadToCourse(uint8_t Speed,int16_t Angle);

MapTouringType CM_LinearMoveToPoint(Point32_t target);
MapTouringType CM_LinearMoveToPoint(Point32_t Target, int32_t speed_max, bool stop_is_needed, bool rotate_is_needed);

MapTouringType CM_MoveToPoint(Point32_t target);

bool CM_Check_is_exploring();
//bool CM_Check_is_exploring();
int CM_Get_grid_index(float position_x, float position_y, uint32_t width, uint32_t height, float resolution, double origin_x, double origin_y);
uint8_t CM_MoveForward(void);

uint8_t CM_Touring(void);

void CM_update_position(uint16_t heading);
void CM_update_map(ActionType action, uint8_t bumper);
void CM_update_map_bumper(ActionType action, uint8_t bumper);

void CM_count_normalize(uint16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y);

int32_t CM_ABS(int32_t A, int32_t B);

int8_t CM_MoveToCell( int16_t x, int16_t y, uint8_t mode, uint8_t length, uint8_t step );

void CM_CorBack(uint16_t dist);

void CM_SetGoHome(uint8_t remote);
void CM_ResetGoHome(void);
void CM_TouringCancel(void);
void CM_SetGyroOffset(int16_t offset);

void CM_SetHome(int32_t x, int32_t y);
void CM_go_home(void);
//void CM_SetStationHome(void);

// This function is for setting the continue point for robot to go after charge.
void CM_SetContinuePoint(int32_t x, int32_t y);

void CM_ResetBoundaryBlocks(void);

void CM_AddTargets(Point16_t zone);
uint8_t CM_IsLowBattery(void);

uint8_t CM_CheckLoopBack(Point16_t target);

void CM_Matrix_Rotate(int32_t x_in, int32_t y_in, int32_t *x_out, int32_t *y_out, double theta);

MapTouringType CM_handleExtEvent(void);

void CM_create_home_boundary(void);


#endif

