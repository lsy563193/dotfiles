#ifndef __CORMOVE_H__
#define __CORMOVE_H__

#include <vector>
#include <deque>
#include <bitset>
#include "mathematics.h"
#include "event_manager.h"

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
	USE_ROS,
	USE_UNCLEAN,
	USE_CLEANED,
	HOMEWAY_NUM
}HomeWay_t;
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

enum {
	EXIT_CLEAN=-1,
	NO_REATH_TARGET=0,
	REATH_TARGET=1,
};

typedef struct {
	Cell_t	pos;
} VWType;

extern float saved_pos_x, saved_pos_y;
extern bool g_move_back_finished;
extern bool g_is_left_start;
extern bool g_from_charger;

extern uint32_t g_wf_start_timer;
extern uint32_t g_wf_diff_timer;
extern bool g_motion_init_succeeded;
extern bool g_go_home_by_remote;
//extern Cell_t g_next_cell;
//extern Cell_t g_target_cell;
extern bool g_resume_cleaning;
extern bool g_have_seen_charger;
extern bool	g_start_point_seen_charger;
extern bool g_exploration_home;
extern std::deque<Cell_t> g_passed_path;

uint8_t angle_to_bumper_status(void);
//int16_t calc_target(int16_t);
int16_t uranged_angle(int16_t angle);

void cm_cleaning(void);


#define WALL_DISTANCE_WHITE_MIN 550
#define WALL_DISTANCE_WHITE_MAX 625
#define WALL_DISTANCE_BLACK_MIN 120
#define WALL_DISTANCE_BLACK_MAX 180
#define WALL_DISTANCE_HIGH_LIMIT 625
#define WALL_DISTANCE_LOW_LIMIT 150

void path_display_path_points(const std::deque<Cell_t>& path);

extern std::vector<Cell_t> g_homes;
extern std::vector<int> g_home_way_list;
extern std::vector<int>::iterator g_home_way_it;
extern Cell_t g_zero_home;
extern int g_wf_reach_count;
extern bool g_check_path_in_advance;
extern bool g_allow_check_path_in_advance;
void path_set_home(const Cell_t& cell);
bool is_fobbit_free();
bool fw_is_time_up();


enum {
	Clean_Mode_Idle = 1,
	Clean_Mode_WallFollow,
	Clean_Mode_Charging,
	Clean_Mode_Go_Charger,
	Clean_Mode_Sleep,
	Clean_Mode_Test,
	Clean_Mode_Remote,
	Clean_Mode_Spot,
	Clean_Mode_Navigation,
	Clean_Mode_Exploration
};
bool cm_is_navigation();
bool cm_is_exploration();
bool cm_is_go_charger();
void cm_set(uint8_t mode);
uint8_t cm_get(void);

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

void cs_work_motor(bool);
uint8_t cs_self_check(uint8_t Check_Code);
uint8_t cm_get(void);
void cm_set(uint8_t mode);
void cs_disable_motors(void);
//bool check_pub_scan();
void cs_paused_setting();

extern double robot_to_wall_distance;
extern float g_back_distance;
extern bool line_is_found;
extern bool g_go_to_charger_back_30cm;
extern bool g_go_to_charger_back_10cm;
extern bool g_go_to_charger_back_0cm;
extern int g_wall_distance;
extern double g_time_straight;
extern double time_start_straight;
extern bool g_slip_backward;
extern double bumper_turn_factor;
extern Cell_t g_zero_home;
#endif
