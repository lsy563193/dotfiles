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

#define Turn_Speed				33

#define Power_On					0x01
#define Power_Off					0x00

#define Error_Code_None			((uint8_t)0x00)
#define Error_Code_LeftWheel	((uint8_t)0x01)
#define Error_Code_RightWheel	((uint8_t)0x02)
#define Error_Code_SideBrush	((uint8_t)0x03)
#define Error_Code_PickUp		((uint8_t)0x04)
#define Error_Code_Cliff		((uint8_t)0x05)
#define Error_Code_Bumper		((uint8_t)0x06)
#define Error_Code_Stuck		((uint8_t)0x07)
#define Error_Code_MainBrush	((uint8_t)0x08)
#define Error_Code_Fan_H		((uint8_t)0x09)
#define Error_Code_WaterTank	((uint8_t)0x0A)
#define Error_Code_BTA			((uint8_t)0x0B)
#define Error_Code_Obs			((uint8_t)0x0C)
#define Error_Code_BatteryLow	((uint8_t)0x0D)
#define Error_Code_Dustbin		((uint8_t)0x0E)
#define Error_Code_Test			((uint8_t)0xff)
#define Error_Code_Test_Null	((uint8_t)0xfe)
#define Error_Code_Encoder		((uint8_t)0xFC)

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

extern uint32_t Rcon_Status;

extern volatile int16_t Left_Wall_BaseLine;
extern volatile int16_t Right_Wall_BaseLine;

void Reset_Work_Time();
uint32_t Get_Work_Time();

void Set_Error_Code(uint8_t Code);
uint8_t Get_Error_Code(void);

void Set_LeftBrush_Stall(uint8_t L);

uint32_t Get_RightWheel_Step(void);
uint32_t Get_LeftWheel_Step(void);
void Reset_RightWheel_Step();
void Reset_LeftWheel_Step();
void Set_Wheel_Step(uint32_t Left,uint32_t Right);
void Reset_Wheel_Step(void);
void Reset_Wall_Step(void);
uint32_t Get_LeftWall_Step(void);
uint32_t Get_RightWall_Step(void);

int32_t Get_Wall_ADC(int8_t dir);

void Set_Dir_Backward(void);
void Set_Dir_Forward(void);

uint8_t Is_Encoder_Fail(void);

void Set_RightBrush_Stall(uint8_t R);

void Wall_Dynamic_Base(uint32_t Cy);
void Set_Wall_Base(int8_t dir, int32_t data);
int32_t Get_Wall_Base(int8_t dir);

void Quick_Back(uint8_t Speed,uint16_t Distance);

void Turn_Left_At_Init(uint16_t speed,int16_t angle);
void Turn_Left(uint16_t speed,int16_t angle);
void Turn_Right(uint16_t speed,int16_t angle);
void Jam_Turn_Left(uint16_t speed,int16_t angle);
void Jam_Turn_Right(uint16_t speed,int16_t angle);
void WF_Turn_Right(uint16_t speed,int16_t angle);

void Set_LeftBrush_Stall(uint8_t L);

void Set_Dir_Backward(void);

uint8_t Is_Encoder_Fail(void);

void Set_RightBrush_Stall(uint8_t R);

void Wall_Dynamic_Base(uint32_t Cy);

void Quick_Back(uint8_t Speed,uint16_t Distance);

//void Turn_Right(uint16_t speed,uint16_t angle);

uint8_t Get_OBS_Status(void);

int32_t Get_FrontOBS(void);
int32_t Get_LeftOBS(void);
int32_t Get_RightOBS(void);

uint8_t Get_Bumper_Status(void);

uint8_t Get_Cliff_Trig(void);

uint8_t Is_AtHomeBase(void);

uint8_t Turn_Connect(void);

void SetHomeRemote(void);

void Reset_HomeRemote(void);

uint8_t IsHomeRemote(void);

uint8_t Is_OBS_Near(void);

uint32_t Get_Rcon_Status(void);

void Set_Rcon_Status(uint32_t code);

void Reset_TempPWM(void);

void Set_Wheel_Speed(uint8_t Left, uint8_t Right);

void Work_Motor_Configure(void);

uint8_t Check_Motor_Current(void);

void Check_SideBrush_Stall(void);

uint8_t Self_Check(uint8_t Check_Code);

uint8_t Check_Bat_Home(void);

uint8_t Check_Bat_Full(void);

uint8_t Check_Bat_Ready_To_Clean(void);

uint8_t Get_Clean_Mode(void);

void Set_VacMode(uint8_t data);

void Set_BLDC_Speed(uint32_t S);

void Set_Vac_Speed(void);

void OBS_Dynamic_Base(uint16_t Cy);
int16_t Get_FrontOBST_Value(void);
int16_t Get_LeftOBST_Value(void);
uint8_t Is_WallOBS_Near(void);
void Adjust_OBST_Value(void);
void Reset_OBST_Value(void);
uint8_t Spot_OBS_Status(void);
uint8_t Get_OBS_Status(void);

void Move_Forward(uint8_t Left_Speed, uint8_t Right_Speed);

uint8_t Get_VacMode(void);

void Switch_VacMode(void);

void Set_Rcon_Remote(uint8_t cmd);

void Reset_Rcon_Remote(void);

uint8_t Get_Rcon_Remote(void);

void Reset_MoveWithRemote(void);

void Set_MoveWithRemote(void);

uint8_t Check_Bat_SetMotors(uint32_t Vacuum_Voltage, uint32_t Side_Brush, uint32_t Main_Brush);

void Reset_WorkTimer(void);

void Reset_Rcon_Status(void);

void Set_Dir_Left(void);

void Set_Dir_Right(void);

void Set_LED(uint16_t G, uint16_t R);

void Stop_Brifly(void);

void Set_MainBrush_PWM(uint16_t PWM);

void Set_SideBrush_PWM(uint16_t L, uint16_t R);

uint8_t Get_LeftBrush_Stall(void);

uint8_t Get_RightBrush_Stall(void);

uint8_t Remote_Key(uint8_t Key);

uint8_t Is_MoveWithRemote(void);

uint8_t Get_Touch_Status(void);

void Reset_Touch(void);

void Set_Touch(void);

void Deceleration(void);

uint8_t Touch_Detect(void);

uint8_t Is_Station(void);

uint8_t Is_ChargerOn(void);

uint8_t Is_Water_Tank(void);

void Set_Clean_Mode(uint8_t mode);

void Beep(uint8_t Sound_Code, int Sound_Time_Count, int Silence_Time_Count, int Total_Time_Count);

void Initialize_Motor(void);

void Disable_Motors(void);

void set_start_charge(void);

void set_stop_charge(void);

bool except_event();
void Set_Gyro_On(void);
bool Wait_For_Gyro_On(void);
void Set_Gyro_Off(void);

void Set_CleanTool_Power(uint8_t vaccum_val,uint8_t left_brush_val,uint8_t right_brush_val,uint8_t main_brush_val);

void control_set(uint8_t type, uint8_t val);

void control_append_crc(void);

void control_stop_all(void);

int control_get_sign(uint8_t* key, uint8_t* sign, uint8_t key_length, int sequence_number);

void Random_Back(void);

void Move_Back(void);

void Cliff_Move_Back(void);

void Set_LeftWheel_Speed(uint8_t speed);

void Set_RightWheel_Speed(uint8_t speed);

int16_t Get_LeftWheel_Speed(void);

int16_t Get_RightWheel_Speed(void);

uint8_t  Check_Bat_Stop();

void Set_Key_Press(uint8_t key);

void Reset_Key_Press(uint8_t key);

uint8_t Get_Key_Press(void);

uint16_t GetBatteryVoltage();

uint8_t Get_Key_Time(uint16_t key);
	
uint8_t IsSendBusy(void);

void SetSendFlag(void);

void ResetSendFlag(void);

uint8_t Is_VirtualWall(void);

uint8_t Is_Bumper_Jamed(void);

void Reset_Bumper_Error(void);

uint8_t Is_Bumper_Fail(void);

uint8_t Is_Turn_Remote(void);

uint8_t Is_Front_Close(void);

void Set_LeftWheel_Step(uint32_t step);

void Set_RightWheel_Step(uint32_t step);

void Set_Direction_Flag(uint8_t flag);

uint8_t Is_Direction_Right();

uint8_t Is_Direction_Left();

uint8_t Get_Direction_Flag();

uint8_t Is_RightWheel_Reach(int32_t step);

uint8_t Is_LeftWheel_Reach(int32_t step);

void Wall_Move_Back(void);

void Reset_Move_Distance(void);

uint8_t Is_Move_Finished(int32_t distance);

uint32_t Get_Move_Distance(void);

void OBS_Turn_Left(uint16_t speed,uint16_t angle);

void OBS_Turn_Right(uint16_t speed,uint16_t angle);

uint8_t Get_Random_Factor();

uint8_t Is_NearStation(void);

void Set_Mobility_Step(uint32_t Steps);

void Reset_Mobility_Step();

uint32_t  Get_Mobility_Step();

void Adjust_OBST_Value();

void Check_Mobility(void);

void Add_Average(uint32_t data);

uint32_t Get_Average_Move(void);

uint32_t Reset_Average_Counter(void);

void Set_Work_Time(time_t );

uint8_t Cliff_Escape(void);

uint8_t Cliff_Event(uint8_t temp_status);

void Reset_VirtualWall();

void Cliff_Turn_Left(uint16_t speed,uint16_t angle);

void Cliff_Turn_Right(uint16_t speed,uint16_t angle);

uint8_t Is_WorkFinish(uint8_t m);

void Set_Room_Mode(uint8_t m);

uint8_t Get_Room_Mode(void);

uint32_t Get_WallAccelerate();

void Reset_WallAccelerate();

uint8_t VirtualWall_TurnRight();

uint8_t VirtualWall_TurnLeft();

void Set_Gyro_Status(void);

void Reset_Gyro_Status(void);

uint8_t Is_Gyro_On(void);

void ladar_gpio(char val);

#if GYRO_DYNAMIC_ADJUSTMENT
void Set_Gyro_Dynamic_On(void);

void Set_Gyro_Dynamic_Off(void);
#endif

int32_t ABS_Minus(int32_t A,int32_t B);
#endif

void Set_Plan_Status(bool Status);
bool Get_Plan_Status(void);

uint8_t Get_Main_PwrByte();
void Set_Main_PwrByte(uint8_t val);
#if MANUAL_PAUSE_CLEANING
void Clear_Manual_Pause(void);
#endif
