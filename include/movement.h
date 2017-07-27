#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__

#include <stdint.h>
#include <time.h>
#include "config.h"
#include "main.h"

#define Brush_Power					128
#define MainBrush_Power				70

#define Direction_Flag_Right		0x01
#define Direction_Flag_Left			0x02

#define Check_Left_Wheel			0x01
#define Check_Right_Wheel			0x02
#define Check_Left_Brush			0x20
#define Check_Right_Brush			0x40

#define Wheel_Stall_Limit			((uint16_t) 580)

#define Check_Main_Brush			0x08
#define Check_Vacuum				0x10
//#define Check_Vacuum				0x20

#define Room_Mode_Auto (uint8_t)0x01
#define Room_Mode_Large (uint8_t)0x00

#define Status_Left_Wall			((uint8_t) 0x01)
#define Status_Left_OBS				((uint8_t) 0x02)
#define Status_Left_OBS_2			((uint8_t) 0x04)

#define Status_Front_OBS			((uint8_t) 0x80)
#define Status_Right_OBS_2			((uint8_t) 0x40)
#define Status_Right_OBS			((uint8_t) 0x20)
#define Status_Right_Wall			((uint8_t) 0x10)

#define Status_Cliff_Left			((uint8_t) 0x01)
#define Status_Cliff_Right			((uint8_t) 0x02)
#define Status_Cliff_Front			((uint8_t) 0x04)
#define Status_Cliff_All			((uint8_t) 0x07)
#define Status_Cliff_LF			((uint8_t) 0x05)
#define Status_Cliff_RF			((uint8_t) 0x06)
#define Status_Cliff_LR			((uint8_t) 0x03)


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

#define RconFrontAll_Home_T		(uint32_t)0x02222220
#define RconAll_Home_T			(uint32_t)0x22222222
#define RconAll_Home_LR			(uint32_t)0x55555555
#define RconFrontAlL_Home_LR	(uint32_t)0x05555550
#define RconAll_Home_TLR		(uint32_t)0x77777777
#define RconFrontAll_Home_TLR	(uint32_t)0x07777770

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

#define Vac_Speed_Max				100 //15500rpm
#define Vac_Speed_Normal			60 //9000rpm
#define Vac_Speed_NormalL			50 //8000rpm

#define Vac_Normal					0
#define Vac_Max						1
#define Vac_Save					2

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

#define Clean_Mode_Userinterface	1
#define Clean_Mode_Spiral			2
#define Clean_Mode_WallFollow		3
#define Clean_Mode_RandomMode		4
#define Clean_Mode_Charging			5
#define Clean_Mode_GoHome			6
#define Clean_Mode_Sleep			7
#define Clean_Mode_SelfCheck		8
#define Clean_Mode_Test				9
#define Clean_Mode_Zigzag			10
#define Clean_Mode_Remote			11
#define Clean_Mode_Spot				12
#define Clean_Mode_Mobility			13
#define Clean_Mode_Navigation		14

#define POWER_ACTIVE 1
#define POWER_DOWN 7


#define Const_160Min_Time			19200 //160 minutes
#define Const_Work_Time				14400 //120 minutes

#define LeftBumperTrig				1
#define RightBumperTrig				2
#define AllBumperTrig				3

#define LeftCliffTrig							1
#define RightCliffTrig						2
#define RightLeftCliffTrig				3
#define FrontCliffTrig						4
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
#define Error_Code_Omni         ((uint8_t)0x16)

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

#define	CTL_WHEEL_LEFT_HIGH 2
#define	CTL_WHEEL_LEFT_LOW  3
#define	CTL_WHEEL_RIGHT_HIGH  4
#define	CTL_WHEEL_RIGHT_LOW 5
#define	CTL_VACCUM_PWR 6
#define	CTL_BRUSH_LEFT 7
#define	CTL_BRUSH_RIGHT 8
#define	CTL_BRUSH_MAIN 9
#define	CTL_BUZZER 10
#define	CTL_MAIN_PWR 11
#define	CTL_CHARGER 12
#define	CTL_LED_RED 13
#define	CTL_LED_GREEN 14
#if __ROBOT_X400
#define	CTL_GYRO 15
#define	CTL_CRC 16
#elif __ROBOT_X900
#define CTL_OMNI_RESET 15
#define CTL_GYRO 16
#define CTL_CMD				17
#define CTL_CRC				18
#endif
#define Direction_Flag_Right 0x01
#define Direction_Flag_Left  0x02

#define STEP_PER_MM  186
#define Cliff_Limit         (int16_t)20

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

extern uint32_t g_rcon_status;

extern volatile int16_t g_left_wall_baseline;
extern volatile int16_t g_right_wall_baseline;

void reset_work_time();
uint32_t get_work_time();

void set_error_code(uint8_t Code);
uint8_t get_error_code(void);
void alarm_error(void);
bool check_error_cleared(uint8_t error_code);

uint32_t get_right_wheel_step(void);
uint32_t get_left_wheel_step(void);
void reset_right_wheel_step();
void reset_left_wheel_step();
void set_wheel_step(uint32_t Left, uint32_t Right);
void reset_wheel_step(void);
void reset_wall_step(void);
uint32_t get_left_wall_step(void);
uint32_t get_right_wall_step(void);

int32_t get_wall_adc(int8_t dir);

void set_dir_backward(void);
void set_dir_forward(void);

uint8_t is_encoder_fail(void);

void set_right_brush_stall(uint8_t R);
void set_left_brush_stall(uint8_t L);

void wall_dynamic_base(uint32_t Cy);
void set_wall_base(int8_t dir, int32_t data);
int32_t get_wall_base(int8_t dir);

void quick_back(uint8_t speed,uint16_t distance);

void turn_left(uint16_t speed, int16_t angle);
void turn_right(uint16_t speed, int16_t angle);
void jam_turn_left(uint16_t speed, int16_t angle);
void jam_turn_right(uint16_t speed, int16_t angle);

void set_dir_backward(void);

uint8_t is_encoder_fail(void);

void set_right_brush_stall(uint8_t R);

void wall_dynamic_base(uint32_t Cy);

//void Turn_Right(uint16_t speed,uint16_t angle);

uint8_t get_obs_status(void);

int32_t get_front_obs(void);
int32_t get_left_obs(void);
int32_t get_right_obs(void);

uint8_t get_bumper_status(void);

uint8_t get_cliff_status(void);

int get_rcon_trig(void);

bool is_on_charger_stub(void);

bool is_direct_charge(void);

void set_home_remote(void);

void reset_home_remote(void);

uint8_t is_home_remote(void);

uint8_t is_obs_near(void);

uint32_t get_rcon_status(void);

void set_rcon_status(uint32_t code);
/*
// Add handling for gyro dynamic adjustment.
// If robot going straight, should turn off gyro dynamic adjustment.
// If robot turning, should turn on gyro dynamic adjustment.
 */
void set_wheel_speed(uint8_t Left, uint8_t Right);

void work_motor_configure(void);

uint8_t check_motor_current(void);

uint8_t check_left_brush_stall(void);
uint8_t check_right_brush_stall(void);

uint8_t self_check(uint8_t Check_Code);

uint8_t check_bat_home(void);

uint8_t check_bat_full(void);

uint8_t check_bat_ready_to_clean(void);

uint8_t get_clean_mode(void);

/*
 * Set the mode for vacuum.
 * The mode should be Vac_Speed_Max/Vac_Speed_Normal/Vac_Speed_NormalL/Vac_Save
 * para
 * mode: Vac_Normal Vac_Max Vac_Save(load mode save last time)
 * save: if save is ture,save this mode,next time clean will reload at interface
 * */
void set_vacmode(uint8_t mode, bool is_save = false);

void set_bldc_speed(uint32_t S);

void set_vac_mode(uint8_t mode);

void set_vac_speed(void);

void obs_dynamic_base(uint16_t Cy);
int16_t get_front_obs_value(void);
int16_t get_left_obs_value(void);
int16_t get_right_obs_value(void);
uint8_t is_wall_obs_near(void);
void adjust_obs_value(void);
void reset_obst_value(void);
uint8_t spot_obs_status(void);
uint8_t get_obs_status(void);

void move_forward(uint8_t Left_Speed, uint8_t Right_Speed);

uint8_t get_vac_mode(void);

/*
 * node:default is not save,go and spod mode is not save, key is save
 */
void switch_vac_mode(bool save);

void set_rcon_remote(uint8_t cmd);

void reset_rcon_remote(void);

uint8_t get_rcon_remote(void);

void reset_move_with_remote(void);

void set_move_with_remote(void);

uint8_t check_bat_set_motors(uint32_t Vacuum_Voltage, uint32_t Side_Brush, uint32_t Main_Brush);

void reset_rcon_status(void);

void set_dir_left(void);

void set_dir_right(void);

void set_led(uint16_t G, uint16_t R);

void stop_brifly(void);

void set_main_brush_pwm(uint16_t PWM);

void set_side_brush_pwm(uint16_t L, uint16_t R);

void set_left_brush_pwm(uint16_t L);

void set_right_brush_pwm(uint16_t R);

uint8_t get_left_brush_stall(void);

uint8_t get_right_brush_stall(void);

uint8_t remote_key(uint8_t Key);

uint8_t is_move_with_remote(void);

uint8_t get_touch_status(void);

void reset_touch(void);

void set_touch(void);

void reset_stop_event_status(void);

uint8_t stop_event(void);

uint8_t is_station(void);

bool is_charge_on(void);

uint8_t is_water_tank(void);

void set_clean_mode(uint8_t mode);

void beep(uint8_t Sound_Code, int Sound_Time_Count, int Silence_Time_Count, int Total_Time_Count);

void initialize_motor(void);

void disable_motors(void);

void set_start_charge(void);

void set_stop_charge(void);

void set_clean_tool_power(uint8_t vaccum_val, uint8_t left_brush_val, uint8_t right_brush_val, uint8_t main_brush_val);

void start_self_check_vacuum(void);

void End_SelfCheck_Vacuumm(void);

void reset_self_check_vacuum_controler(void);

void control_set(uint8_t type, uint8_t val);

void control_append_crc(void);

void control_stop_all(void);

int control_get_sign(uint8_t* key, uint8_t* sign, uint8_t key_length, int sequence_number);

void random_back(void);

void move_back(void);

void cliff_move_back(void);

void set_left_wheel_speed(uint8_t speed);

void set_right_wheel_speed(uint8_t speed);

int16_t get_left_wheel_speed(void);

int16_t get_right_wheel_speed(void);

uint8_t  check_bat_stop();

void set_key_press(uint8_t key);

void reset_key_press(uint8_t key);

uint8_t get_key_press(void);

uint16_t get_battery_voltage();

uint8_t is_flag_set(void);

void set_send_flag(void);

void reset_send_flag(void);

uint8_t is_virtual_wall_(void);

uint8_t is_bumper_jamed(void);

void reset_bumper_error(void);

uint8_t is_bumper_fail(void);

uint8_t is_turn_remote(void);

uint8_t is_front_close(void);

void set_left_wheel_step(uint32_t step);

void set_right_wheel_step(uint32_t step);

void set_direction_flag(uint8_t flag);

uint8_t is_direction_right();

uint8_t is_direction_left();

uint8_t get_direction_flag();

uint8_t is_right_wheel_reach(int32_t step);

uint8_t is_left_wheel_reach(int32_t step);

void wall_move_back(void);

void reset_move_distance(void);

uint8_t is_move_finished(int32_t distance);

uint32_t get_move_distance(void);

void obs_turn_left(uint16_t speed, uint16_t angle);

void obs_turn_right(uint16_t speed, uint16_t angle);

uint8_t get_random_factor();

uint8_t is_near_station(void);

void set_mobility_step(uint32_t Steps);

void reset_mobility_step();

void clear_reset_mobility_step();

uint32_t  get_mobility_step();

void adjust_obs_value();

void check_mobility(void);

void add_average(uint32_t data);

uint32_t get_average_move(void);

uint32_t reset_average_counter(void);

uint8_t cliff_escape(void);

uint8_t cliff_event(uint8_t temp_status);

void reset_virtual_wall();

void cliff_turn_left(uint16_t speed, uint16_t angle);

void cliff_turn_right(uint16_t speed, uint16_t angle);

uint8_t is_work_finish(uint8_t m);

void set_room_mode(uint8_t m);

uint8_t get_room_mode(void);

uint32_t get_wall_accelerate();

void reset_wall_accelerate();

uint8_t virtual_wall_turn_right();

uint8_t virtual_wall_turn_left();

int32_t abs_minus(int32_t A, int32_t B);

void set_plan_status(uint8_t Status);
uint8_t get_plan_status(void);

uint8_t get_main_pwr_byte();
void set_main_pwr_byte(uint8_t val);

void set_sleep_mode_flag();
uint8_t get_sleep_mode_flag();
void reset_sleep_mode_flag();

uint8_t get_self_check_vacuum_status(void);

//#if MANUAL_PAUSE_CLEANING
void clear_manual_pause(void);
//#endif

void beep_for_command(bool valid);

void reset_sp_turn_count();
int32_t get_sp_turn_count();
void add_sp_turn_count();

// time_ms is used for both LED_FLASH type and LED_BREATH type, the default value is for LED_BREATH.
void set_led_mode(uint8_t type, uint8_t color, uint16_t time_ms = 3000);

#endif
