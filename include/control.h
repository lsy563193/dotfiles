#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>

#include "config.h"

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

#define Cliff_PWM_Duration			((uint16_t) 0x000D)	//set Cliff PWM Duration = 13
#define Obstacle_PWM_Duration		((uint16_t) 0x000D)	//set Obstacle PWM Duration =13

#define Charge_Home_Left			((uint8_t) 0x50)
#define Charge_Home_Right			((uint8_t) 0x02)
#define Charge_Home_Top				((uint8_t) 0x0c)
#define Vitual_Wall_Code			((uint8_t) 0x24)

#define RconL_HomeL					((uint32_t) 0x00000080)
#define RconL_HomeR					((uint32_t) 0x00000040)

#define RconFL_HomeL				((uint32_t) 0x00000020)
#define RconFL_HomeR				((uint32_t) 0x00000010)

#define RconFR_HomeL				((uint32_t) 0x00000008)
#define RconFR_HomeR				((uint32_t) 0x00000004)

#define RconR_HomeL					((uint32_t) 0x00000002)
#define RconR_HomeR					((uint32_t) 0x00000001)

#define RconR_HomeT					((uint32_t) 0x00000100)
#define RconFR_HomeT				((uint32_t) 0x00000200)
#define RconFL_HomeT				((uint32_t) 0x00000400)
#define RconL_HomeT					((uint32_t) 0x00000800)

#define RconBR_HomeL				((uint32_t) 0x00001000)
#define RconBR_HomeR				((uint32_t) 0x00002000)
#define RconBR_HomeT				((uint32_t) 0x00004000)

#define RconBL_HomeL				((uint32_t) 0x00010000)
#define RconBL_HomeR				((uint32_t) 0x00020000)
#define RconBL_HomeT				((uint32_t) 0x00040000)

#define RconR_Wall					((uint32_t) 0x00000000)
#define RconFR_Wall					((uint32_t) 0x00000000)
#define RconFL_Wall					((uint32_t) 0x00000000)
#define RconL_Wall					((uint32_t) 0x00000000)
#define RconBL_Wall					((uint32_t) 0x00000000)
#define RconBR_Wall					((uint32_t) 0x00000000)

#define Remote_Clean				((uint32_t) 0X02AA22DD)
#define Remote_Forward				((uint32_t) 0X02AA55AA)
#define Remote_Left					((uint32_t) 0X02AA33CC)
#define Remote_Max					((uint32_t) 0X02AA6699)
#define Remote_Right				((uint32_t) 0X02AA44BB)
#define Remote_Home					((uint32_t) 0X02AA8877)
#define Remote_Random				((uint32_t) 0X02AA9966)
#define Remote_Spot					((uint32_t) 0X02AA7788)

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
#define Low_Battery_Limit			((uint16_t) 1200)
#define MainBrush_Stall_Current		((uint16_t) 1000)

#define Clean_Vac_Power				120000
#define Home_Vac_Power				40000

#define Vac_Speed_Max				1550 //775=15500rpm=1000PA
#define Vac_Speed_Normal			900 //9000rpm
#define Vac_Speed_NormalL			800 //8000rpm

#define Vac_Normal					0
#define Vac_Max						1

#define Clean_MainBrush_Power		85000
#define Home_MainBrush_Power		40000

#define Clean_SideBrush_Power		80000
#define Home_SideBrush_Power		40000
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


#define Const_160Min_Time			19200 //160 minutes
#define Const_Work_Time				14400 //120 minutes

#define LeftBumperTrig				1
#define RightBumperTrig				2
#define AllBumperTrig				3

#define Turn_Speed					33

#define Power_On					0x01
#define Power_Off					0x00

#define Error_Code_Bumper			0x02
#define Error_Code_Cliff			0x02

#define Error_Code_Encoder			0x0C


#define Display_Low					5

#define	CTL_WHEEL_LEFT_HIGH 0
#define	CTL_WHEEL_LEFT_LOW  1
#define	CTL_WHEEL_RIGHT_HIGH  2
#define	CTL_WHEEL_RIGHT_LOW 3
#define	CTL_VACCUM_PWR 4
#define	CTL_BRUSH_LEFT 5
#define	CTL_BRUSH_RIGHT 6
#define	CTL_BRUSH_MAIN 7
#define	CTL_BUZZER 8
#define	CTL_MAIN_PWR 9
#define	CTL_CHARGER 10
#define	CTL_LED_RED 11
#define	CTL_LED_GREEN 12
#define	CTL_GYRO 13

void control_set_wheel_speed(int16_t left, int16_t right);
void control_set_wheel_left_speed(int16_t val);
void control_set_wheel_right_speed(int16_t val);
void control_set_vaccum_pwr(uint8_t val);
void control_set_brush_left(uint8_t val);
void control_set_brush_right(uint8_t val);
void control_set_brush_main(uint8_t val);
void control_set_buzzer(uint8_t val);
void control_set_main_pwr(uint8_t val);
void control_set_led_red(uint8_t val);
void control_set_led_green(uint8_t val);
void control_set_gyro(uint8_t state, uint8_t calibration);
void control_set(uint8_t type, uint8_t val);
void control_set_cleantool_pwr(uint8_t val);

#endif
