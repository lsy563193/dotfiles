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

#define Direction_Flag_Right		0x01
#define Direction_Flag_Left			0x02

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

#define CLIFF_LIMIT  60
#define BLOCK_LEFT				((uint8_t) 0x01)
#define BLOCK_RIGHT			((uint8_t) 0x02)
#define BLOCK_FRONT			((uint8_t) 0x04)
#define BLOCK_ALL			((uint8_t) 0x07)

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

#if __ROBOT_X900

#define RconBL_HomeL		(uint32_t)0x40000000
#define RconBL_HomeT		(uint32_t)0x20000000
#define RconBL_HomeR		(uint32_t)0x10000000

#define RconL_HomeL			(uint32_t)0x04000000
#define RconL_HomeT			(uint32_t)0x02000000
#define RconL_HomeR			(uint32_t)0x01000000

#define RconFL2_HomeL		(uint32_t)0x00400000
#define RconFL2_HomeT		(uint32_t)0x00200000
#define RconFL2_HomeR		(uint32_t)0x00100000

#define RconFL_HomeL		(uint32_t)0x00040000
#define RconFL_HomeT		(uint32_t)0x00020000
#define RconFL_HomeR		(uint32_t)0x00010000

#define RconFR_HomeL		(uint32_t)0x00004000
#define RconFR_HomeT		(uint32_t)0x00002000
#define RconFR_HomeR		(uint32_t)0x00001000

#define RconFR2_HomeL		(uint32_t)0x00000400
#define RconFR2_HomeT		(uint32_t)0x00000200
#define RconFR2_HomeR		(uint32_t)0x00000100

#define RconR_HomeL			(uint32_t)0x00000040
#define RconR_HomeT			(uint32_t)0x00000020
#define RconR_HomeR			(uint32_t)0x00000010

#define RconBR_HomeL		(uint32_t)0x00000004
#define RconBR_HomeT		(uint32_t)0x00000002
#define RconBR_HomeR		(uint32_t)0x00000001

#define RconAll_Home_T			(uint32_t)0x22222222
#define RconAll_Home_LR			(uint32_t)0x55555555
#define RconAll_Home_TLR		(uint32_t)0x77777777
#define RconFrontAll_Home_T		(uint32_t)0x02222220
#define RconFrontAll_Home_LR	(uint32_t)0x05555550
#define RconFrontAll_Home_TLR	(uint32_t)0x07777770
#define RconFront_Home_T		(uint32_t)0x00222200
#define RconFront_Home_LR		(uint32_t)0x00555500
#define RconFront_Home_TLR		(uint32_t)0x00777700

#ifdef VIRTUAL_WALL

#define RconBL_Wall          	(uint16_t)0x8000
#define RconL_Wall           	(uint16_t)0x4000
#define RconFL2_Wall           	(uint16_t)0x2000
#define RconFL_Wall           	(uint16_t)0x1000
#define RconFR_Wall           	(uint16_t)0x0800
#define RconFR2_Wall           	(uint16_t)0x0400
#define RconR_Wall           	(uint16_t)0x0200
#define RconBR_Wall           	(uint16_t)0x0100
#define RconBL_Wall_T           (uint16_t)0x0080
#define RconL_Wall_T           	(uint16_t)0x0040
#define RconFL2_Wall_T          (uint16_t)0x0020
#define RconFL_Wall_T           (uint16_t)0x0010
#define RconFR_Wall_T           (uint16_t)0x0008
#define RconFR2_Wall_T          (uint16_t)0x0004
#define RconR_Wall_T           	(uint16_t)0x0002
#define RconBR_Wall_T           (uint16_t)0x0001

#else

#define RconFL_Wall          	(uint16_t)0x0000
#define RconFL_Wall_T         (uint16_t)0x0000
#define RconFR_Wall          	(uint16_t)0x0000
#define RconFR_Wall_T         (uint16_t)0x0000
#define RconFL2_Wall         	(uint16_t)0x0000
#define RconFL2_Wall_T        (uint16_t)0x0000
#define RconFR2_Wall         	(uint16_t)0x0000
#define RconFR2_Wall_T        (uint16_t)0x0000
#define RconL_Wall           	(uint16_t)0x0000
#define RconL_Wall_T          (uint16_t)0x0000
#define RconR_Wall           	(uint16_t)0x0000
#define RconR_Wall_T          (uint16_t)0x0000
#define RconBL_Wall          	(uint16_t)0x0000
#define RconBL_Wall_T         (uint16_t)0x0000
#define RconBR_Wall          	(uint16_t)0x0000
#define RconBR_Wall_T         (uint16_t)0x0000
#endif

#else

#define Rcon_Wall 					((uint32_t) 0x00000008)
#define Rcon_HomeL 					((uint32_t) 0x00000004)
#define Rcon_HomeT 					((uint32_t) 0x00000002)
#define Rcon_HomeR 					((uint32_t) 0x00000001)

#define RconL_HomeL					((uint32_t) 0x00004000)
#define RconL_HomeR					((uint32_t) 0x00001000)

#define RconFL_HomeL				((uint32_t) 0x00400000)
#define RconFL_HomeR				((uint32_t) 0x00100000)

#define RconFR_HomeL				((uint32_t) 0x00040000)
#define RconFR_HomeR				((uint32_t) 0x00010000)

#define RconR_HomeL					((uint32_t) 0x00000400)
#define RconR_HomeR					((uint32_t) 0x00000100)

#define RconR_HomeT					((uint32_t) 0x00000200)
#define RconFR_HomeT				((uint32_t) 0x00020000)
#define RconFL_HomeT				((uint32_t) 0x00200000)
#define RconL_HomeT					((uint32_t) 0x00002000)

#define RconBR_HomeL				((uint32_t) 0x00000004)
#define RconBR_HomeR				((uint32_t) 0x00000001)
#define RconBR_HomeT				((uint32_t) 0x00000002)

#define RconBL_HomeL				((uint32_t) 0x00000040)
#define RconBL_HomeR				((uint32_t) 0x00000010)
#define RconBL_HomeT				((uint32_t) 0x00000020)

#define RconR_Wall					((uint32_t) 0x00000700)
#define RconFR_Wall					((uint32_t) 0x00070000)
#define RconFL_Wall					((uint32_t) 0x00700000)
#define RconL_Wall					((uint32_t) 0x00007000)
#define RconBL_Wall					((uint32_t) 0x00000070)
#define RconBR_Wall					((uint32_t) 0x00000007)
#endif

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

#define Error_Code_None			((uint8_t)0x00)
#define Error_Code_LeftWheel	((uint8_t)0x01)
#define Error_Code_RightWheel	((uint8_t)0x02)
#define Error_Code_LeftBrush	((uint8_t)0x03)
#define Error_Code_RightBrush	((uint8_t)0x04)
#define Error_Code_PickUp		((uint8_t)0x05)
#define Error_Code_Cliff		((uint8_t)0x06)
#define Error_Code_Bumper		((uint8_t)0x07)
#define Error_Code_Stuck		((uint8_t)0x08)
#define Error_Code_MainBrush	((uint8_t)0x09)
#define Error_Code_Fan_H		((uint8_t)0x0A)
#define Error_Code_WaterTank	((uint8_t)0x0B)
#define Error_Code_BTA			((uint8_t)0x0C)
#define Error_Code_Obs			((uint8_t)0x0D)
#define Error_Code_BatteryLow	((uint8_t)0x0E)
#define Error_Code_Dustbin		((uint8_t)0x0F)
#define Error_Code_Gyro			((uint8_t)0x10)
#define Error_Code_Encoder		((uint8_t)0x11)
#define Error_Code_Slam			((uint8_t)0x12)
#define Error_Code_Laser		((uint8_t)0x13)
#define Error_Code_Test			((uint8_t)0x14)
#define Error_Code_Test_Null	((uint8_t)0x15)
#define Error_Code_Omni			((uint8_t)0x16)

#define Display_Full				4
#define Display_Low					5

#if __ROBOT_X900
#define KEY_CLEAN 1
#else
#define KEY_CLEAN 0x02
#endif
#define KEY_CLOCK 0x01
#define KEY_SPOT  0x04
#define KEY_HOME  0x08
#define KEY_PLAN  0x10


#define Direction_Flag_Right 0x01
#define Direction_Flag_Left  0x02

#define STEP_PER_MM  186

#define Two_Hours         7200

#define DUMMY_DOWNLINK_OFFSET		2
#define KEY_DOWNLINK_OFFSET			9
#define SEQUENCE_DOWNLINK_OFFSET	7

#define DUMMY_DOWNLINK_LENGTH		5
#define SEQUENCE_DOWNLINK_LENGTH	2
#define KEY_DOWNLINK_LENGTH			8

#define KEY_UPLINK_OFFSET			36
#define KEY_UPLINK_LENGTH			16

#define CMD_UPLINK_OFFSET			53

#define CMD_KEY1					0x40
#define CMD_KEY2					0x41
#define CMD_KEY3					0x42
#define CMD_ID						0x43

#define CMD_ACK						0x23
#define CMD_NCK						0x25

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

// for wheel direction
#define FORWARD						0
#define BACKWARD					1

//regulator type
#define REG_TYPE_NONE			0
#define REG_TYPE_WALLFOLLOW		1
#define REG_TYPE_LINEAR			2
#define REG_TYPE_TURN			3
#define REG_TYPE_BACK			4
#define REG_TYPE_CURVE			5

extern uint32_t g_rcon_status;

extern int16_t g_obs_left_baseline;
extern int16_t g_obs_front_baseline;
extern int16_t g_obs_right_baseline;

extern volatile int16_t g_left_wall_baseline;
extern volatile int16_t g_right_wall_baseline;

struct pid_struct
{
	float delta;
	float delta_sum;
	float delta_last;
	float target_speed;
	float actual_speed;
	float last_target_speed;
	uint8_t last_reg_type;
	float variation;
};

struct pid_argu_struct
{
	uint8_t reg_type; // Regulator type
	float Kp;
	float Ki;
	float Kd;
};

#define RCON_SENSOR_FLFR 0 //define the rcon sensor angle on robotbase (degree)
#define RCON_SENSOR_FLFR2 30
#define RCON_SENSOR_LR 60
typedef struct {
	int32_t x;
	int32_t y;
	int16_t sensor_angle;
}Rcon_Point_t;

extern struct pid_argu_struct argu_for_pid;
extern struct pid_struct left_pid, right_pid;

void reset_work_time();
uint32_t get_work_time();

void set_error_code(uint8_t Code);
uint8_t get_error_code(void);
void alarm_error(void);
bool check_error_cleared(uint8_t error_code);

int32_t get_right_wheel_step(void);
int32_t get_left_wheel_step(void);
void reset_wheel_step(void);
bool is_decelerate_wall(void);

int32_t get_wall_adc(int8_t dir);

void set_dir_backward(void);
void set_dir_forward(void);

uint8_t is_encoder_fail(void);


void wall_dynamic_base(uint32_t Cy);
void set_wall_base(int8_t dir, int32_t data);
int32_t get_wall_base(int8_t dir);

void quick_back(uint8_t speed,uint16_t distance);

void set_dir_backward(void);

uint8_t is_encoder_fail(void);

int rcon_get_trig(void);

bool is_on_charger_stub(void);

bool is_direct_charge(void);

/*
// Add handling for gyro dynamic adjustment.
// If robot going straight, should turn off gyro dynamic adjustment.
// If robot turning, should turn on gyro dynamic adjustment.
 */
void set_argu_for_pid(uint8_t reg_type, float Kp, float Ki, float Kd);
void wheels_pid(void);
void set_wheel_speed(uint8_t Left, uint8_t Right, uint8_t reg_type = REG_TYPE_NONE, float PID_p = 1, float PID_i = 0, float PID_d = 0);

void work_motor_configure(void);

uint8_t self_check(uint8_t Check_Code);

uint8_t cm_get(void);

void obs_dynamic_base(uint16_t Cy);
int16_t get_front_obs_trig_value(void);
int16_t get_left_obs_trig_value(void);
int16_t get_right_obs_trig_value(void);
uint8_t get_obs_status(int16_t left_obs_offset = 0, int16_t front_obs_offset = 0, int16_t right_obs_offset = 0);

int16_t get_front_obs(void);
int16_t get_left_obs(void);
int16_t get_right_obs(void);

void move_forward(uint8_t Left_Speed, uint8_t Right_Speed);

void set_dir_left(void);

void set_dir_right(void);

void set_led(uint16_t G, uint16_t R);

void stop_brifly(void);

uint8_t remote_key(uint8_t Key);

void reset_stop_event_status(void);

uint8_t is_water_tank(void);

void cm_set(uint8_t mode);

void beep(uint8_t Sound_Code, int Sound_Time_Count, int Silence_Time_Count, int Total_Time_Count);

void disable_motors(void);

void set_start_charge(void);

void set_stop_charge(void);

void start_self_check_vacuum(void);

void reset_self_check_vacuum_controler(void);

void control_set(uint8_t type, uint8_t val);

uint8_t control_get(uint8_t seq);

int control_get_sign(uint8_t* key, uint8_t* sign, uint8_t key_length, int sequence_number);

void set_left_wheel_speed(uint8_t speed);

void set_right_wheel_speed(uint8_t speed);

int16_t get_left_wheel_speed(void);

int16_t get_right_wheel_speed(void);

void set_send_flag(void);

void reset_send_flag(void);

void set_direction_flag(uint8_t flag);

void reset_mobility_step();

void clear_reset_mobility_step();

int32_t abs_minus(int32_t A, int32_t B);

uint8_t get_main_pwr_byte();
void set_main_pwr_byte(uint8_t val);

void set_sleep_mode_flag();
uint8_t get_sleep_mode_flag();
void reset_sleep_mode_flag();

uint8_t get_self_check_vacuum_status(void);

void beep_for_command(bool valid);

void reset_sp_turn_count();
int32_t get_sp_turn_count();
void add_sp_turn_count();

// time_ms is used for both LED_FLASH type and LED_BREATH type, the default value is for LED_BREATH.
void set_led_mode(uint8_t type, uint8_t color, uint16_t time_ms = 3000);

int16_t get_front_acc();
int16_t get_left_acc();
int16_t get_right_acc();
int16_t get_front_init_acc();
int16_t get_left_init_acc();
int16_t get_right_init_acc();
uint8_t check_tilt();
void set_tilt_status(uint8_t status);
uint8_t get_tilt_status();

bool check_pub_scan();

uint8_t is_robot_slip();
bool is_clean_paused();
void reset_clean_paused();

bool check_laser_stuck();

uint8_t get_laser_status();
#endif
