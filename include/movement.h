#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__

#include <algorithm>
#include <stdint.h>
#include <time.h>
#include "config.h"
#include "main.h"
#include "mathematics.h"

#define Brush_Power					128
#define MainBrush_Power				70

#define Check_Left_Wheel			0x01
#define Check_Right_Wheel			0x02
#define Check_Left_Brush			0x20
#define Check_Right_Brush			0x40

#define Wheel_Stall_Limit			((uint16_t) 750)//origin 580

#define Check_Main_Brush			0x08
#define Check_Vacuum				0x10
//#define Check_Vacuum				0x20

#define Room_Mode_Auto (uint8_t)0x01
#define Room_Mode_Large (uint8_t)0x00

#define Status_Left_Wall			((uint8_t) 0x01)
//#define Status_Left_OBS				((uint8_t) 0x02)
#define Status_Left_OBS_2			((uint8_t) 0x04)

//#define Status_Front_OBS			((uint8_t) 0x80)
#define Status_Right_OBS_2			((uint8_t) 0x40)
//#define Status_Right_OBS			((uint8_t) 0x20)
#define Status_Right_Wall			((uint8_t) 0x10)

//#define Status_Cliff_Left			((uint8_t) 0x01)
//#define Status_Cliff_Right			((uint8_t) 0x02)
//#define Status_Cliff_Front			((uint8_t) 0x04)
//#define Status_Cliff_All			((uint8_t) 0x07)
#define Status_Cliff_LF			((uint8_t) 0x08)
#define Status_Cliff_RF			((uint8_t) 0x10)
#define Status_Cliff_LR			((uint8_t) 0x20)


#define Cliff_PWM_Duration			((uint16_t) 0x000D)	//set Cliff PWM Duration = 13
#define Obstacle_PWM_Duration		((uint16_t) 0x000D)	//set Obstacle PWM Duration =13

#define Charge_Home_Left			((uint8_t) 0x50)
#define Charge_Home_Right			((uint8_t) 0x02)
#define Charge_Home_Top				((uint8_t) 0x0c)
#define Vitual_Wall_Code			((uint8_t) 0x24)

#define Remote_All					((uint8_t) 0xFF)
#define Remote_Forward				((uint8_t) 0x80)
#define Remote_Left					((uint8_t) 0x40)
#define Remote_Right				((uint8_t) 0x20)
#define Remote_Backward				((uint8_t) 0x10)
#define Remote_Max					((uint8_t) 0x10)
#define Remote_Clean				((uint8_t) 0x08)
#define Remote_Home					((uint8_t) 0x04)
#define Remote_Wall_Follow			((uint8_t) 0x02)
#define Remote_Spot					((uint8_t) 0x01)

#define Rcon_Virtualwall			((uint8_t) 0xaa)
#define Rcon_Charger_Station_Left	((uint8_t) 0x02)
#define Rcon_Charger_Station_Right	((uint8_t) 0x50)
#define Rcon_Charger_Station_Top	((uint8_t) 0x0c)

#define Virtualwall_Left			((uint8_t) 0x01)
#define Virtualwall_Right			((uint8_t) 0x02)

#define ReferenceVoltage			((uint16_t) 330)
#define Charge_Current_800mA		((uint16_t) 800)
#define Charge_Current_500mA		((uint16_t) 400)
#define Left_Brush_Limit			((uint16_t) 900)
#define Right_Brush_Limit			((uint16_t) 900)
#define MainBrush_Stall_Current		((uint16_t) 1000)

#define Clean_Vac_Power				120000
#define Home_Vac_Power				40000

#define Clean_MainBrush_Power		8500
#define Home_MainBrush_Power		4000

#define Clean_SideBrush_Power		8000
#define Home_SideBrush_Power		4000
//brush actual 12V

#define Charger_Detected_Voltage	(uint16_t)1700
#define Charger_OverLoad_Voltage	(uint16_t)1900

#define Wall_High_Limit				400
#define Wall_Low_Limit				40
#define MainBrush_Slow_Limit		5

#define Display_Clean				0
#define Display_Wall				1
#define Display_Zizag				2
#define Display_Remote				3

#define POWER_ACTIVE 1
#define POWER_DOWN 7


#define Const_160Min_Time			19200 //160 minutes
#define Const_Work_Time				14400 //120 minutes

//#define LeftBumperTrig				1
//#define RightBumperTrig				2
//#define LidarBumperTrig				4
//#define AllBumperTrig				3

//#define LeftCliffTrig							1
//#define RightCliffTrig						2
#define RightLeftCliffTrig				3
//#define FrontCliffTrig						4
#define LeftFrontCliffTrig				5
#define RightFrontCliffTrig				6
/*
typedef enum{
	TRIG_NULL 				=0,
	TRIG_LEFT 				=1,
	TRIG_RIGHT 				=2,
	TRIG_LEFT_RIGHT		=3,
	TRIG_FRONT				=4,
	TRIG_LEFT_FRONT		=5,
	TRIG_RIGHT_FRONT	=6,
};
*/

#define Turn_Speed				33

#define Power_On					0x01
#define Power_Off					0x00

#define Display_Full				4
#define Display_Low					5


#define KEY_CLOCK 0x01
#define KEY_SPOT  0x04
#define KEY_HOME  0x08
#define KEY_PLAN  0x10


#define Direction_Flag_Right 0x01
#define Direction_Flag_Left  0x02

#define STEP_PER_MM  186

#define Two_Hours         7200

// For beep_for_command()
#define VALID						true
#define INVALID						false

// For LED control
// LED type
#define LED_STEADY					0
#define LED_FLASH					1
#define LED_BREATH					2
// LED color
#define LED_OFF						0
#define LED_GREEN					1
#define LED_ORANGE					2
#define LED_RED						3

//for tilt detct
#define TILT_COUNT_REACH			50
#define DIF_TILT_X_VAL				170
#define DIF_TILT_Y_VAL				170
#define DIF_TILT_Z_VAL				70
#define FRONT_TILT_LIMIT			120
#define LEFT_TILT_LIMIT				120
#define RIGHT_TILT_LIMIT			120
#define TILT_RIGHT					0x1
#define TILT_FRONT					0x2
#define TILT_LEFT					0x4

extern uint32_t g_rcon_status;

#define RCON_SENSOR_FLFR 0 //define the rcon sensor angle on robotbase (degree)
#define RCON_SENSOR_FLFR2 30
#define RCON_SENSOR_LR 60
typedef struct {
	int32_t x;
	int32_t y;
	int16_t sensor_angle;
}Rcon_Point_t;

void reset_work_time();
uint32_t get_work_time();

void error_set(uint8_t Code);
uint8_t error_get(void);
void error_alarm(void);
bool error_clear(uint8_t error_code);

void wheel_reset_step(void);


uint8_t is_encoder_fail(void);

void quick_back(uint8_t speed,uint16_t distance);

uint8_t is_encoder_fail(void);

void cs_work_motor(void);

uint8_t cs_self_check(uint8_t Check_Code);

uint8_t cm_get(void);

void cm_set(uint8_t mode);

void cs_disable_motors(void);

void reset_sp_turn_count();
int32_t get_sp_turn_count();
void add_sp_turn_count();

bool check_pub_scan();

bool cs_is_paused();
void cs_paused_setting();

#endif
