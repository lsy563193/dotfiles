#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__

#include <stdint.h>

void Set_Error_Code(uint8_t Code);

void Set_LeftBrush_Stall(uint8_t L);

uint32_t Get_RightWheel_Step(void);
uint32_t Get_LeftWheel_Step(void);

void Set_Wheel_Step(uint32_t Left,uint32_t Right);

int32_t Get_Wall_ADC(void);

void Set_Dir_Backward(void);
void Set_Dir_Forward(void);

uint8_t Is_Encoder_Fail(void);

void Set_RightBrush_Stall(uint8_t R);

void Wall_Dynamic_Base(uint32_t Cy);

void Quick_Back(uint8_t Speed,uint16_t Distance);

void Turn_Left(uint16_t speed,uint16_t angle);
void Turn_Right(uint16_t speed,uint16_t angle);

void Set_Error_Code(uint8_t code);
void Set_LeftBrush_Stall(uint8_t L);
uint32_t Get_RightWheel_Step(void);
uint32_t Get_LeftWheel_Step(void);

void Set_Wheel_Step(uint32_t Left,uint32_t Right);

int32_t Get_Wall_ADC(void);

void Set_Dir_Backward(void);

uint8_t Is_Encoder_Fail(void);

void Set_RightBrush_Stall(uint8_t R);

void Wall_Dynamic_Base(uint32_t Cy);

void Quick_Back(uint8_t Speed,uint16_t Distance);

void Turn_Right(uint16_t speed,uint16_t angle);

uint8_t Get_OBS_Status(void);

int32_t Get_FrontOBS(void);
int32_t Get_LeftOBS(void);

uint8_t Get_Bumper_Status(void);

uint8_t Get_Cliff_Trig(void);

uint8_t Is_AtHomeBase(void);

void SetHomeRemote(void);

uint8_t Is_OBS_Near(void);

uint32_t Get_Rcon_Status(void);

void Set_Rcon_Status(uint32_t code);

void Reset_TempPWM(void);

void Set_Wheel_Speed(uint8_t Left, uint8_t Right);

void Work_Motor_Configure(void);

uint8_t Check_Motor_Current(void);

uint8_t Self_Check(uint8_t Check_Code);

uint8_t Check_Bat_Home(void);

uint8_t Get_Clean_Mode(void);

void Set_VacMode(uint8_t data);

void Set_BLDC_Speed(uint32_t S);

void Set_Vac_Speed(void);

void OBS_Dynamic_Base(uint16_t Cy);

int16_t Get_FrontOBST_Value(void);
int16_t Get_LeftOBST_Value(void);
uint8_t Is_WallOBS_Near(void);

void Move_Forward(uint8_t Left_Speed, uint8_t Right_Speed);

uint8_t Get_VacMode(void);

void Switch_VacMode(void);

uint32_t Get_Rcon_Remote(void);

void Reset_MoveWithRemote(void);

uint8_t Check_Bat_SetMotors(uint32_t Vacuum_Voltage, uint32_t Side_Brush, uint32_t Main_Brush);

void Reset_WorkTimer(void);

void Reset_Rcon_Status(void);

void Display_Battery_Status(uint8_t temp);

void Set_Dir_Left(void);

void Set_Dir_Right(void);

void Set_LED(uint16_t G, uint16_t R);

void Stop_Brifly(void);

void Set_SideBrush_PWM(uint16_t L, uint16_t R);

uint8_t Get_LeftBrush_Stall(void);

uint8_t Get_RightBrush_Stall(void);

uint8_t Remote_Key(uint32_t Key);

void Reset_Touch(void);

void Set_Touch(void);

void Deceleration(void);

uint8_t Touch_Detect(void);

uint8_t Is_Station(void);

uint8_t Is_ChargerOn(void);

uint8_t Is_Water_Tank(void);

void Set_Clean_Mode(uint8_t mode);

void Beep(uint8_t Sound);

void Disable_Motors(void);

#endif
