#include <stdint.h>

#include "control.h"
#include "robot.hpp"

#include "gyro.h"
#include "movement.h"

static int16_t Left_OBSTrig_Value = 500;
static int16_t Front_OBSTrig_Value = 500;
static int16_t Right_OBSTrig_Value = 500;

static uint8_t wheel_left_direction = 0;
static uint8_t wheel_right_direction = 0;

void Set_Error_Code(uint8_t code)
{
	code = code;
}

void Set_LeftBrush_Stall(uint8_t L)
{
	L = L;
}

uint32_t Get_RightWheel_Step(void)
{
	return 0;
}

uint32_t Get_LeftWheel_Step(void)
{
	return 0;
}

void Set_Wheel_Step(uint32_t Left, uint32_t Right)
{
	Left = Left;
	Right = Right;
}

int32_t Get_Wall_ADC(void)
{
	return 0;
}

void Set_Dir_Backward(void)
{
	wheel_left_direction = 1;
	wheel_right_direction = 1;
}

void Set_Dir_Forward(void)
{
	wheel_left_direction = 0;
	wheel_right_direction = 0;
}

uint8_t Is_Encoder_Fail(void)
{
	return 0;
}

void Set_RightBrush_Stall(uint8_t R)
{
	R = R;
}

void Wall_Dynamic_Base(uint32_t Cy)
{
	Cy = Cy;
}

void Quick_Back(uint8_t Speed, uint16_t Distance)
{
	Speed = Speed;
	Distance = Distance;
}

void Turn_Left(uint16_t speed, uint16_t angle)
{
	int16_t target_angle;
	uint16_t gyro_angle;

	gyro_angle = Gyro_GetAngle(0);

	target_angle = gyro_angle + angle;
	if (target_angle >= 3600) {
		target_angle = target_angle - 3600;
	}

	wheel_left_direction = 1;
	wheel_right_direction = 0;

	Set_Wheel_Speed(speed, speed);

	printf("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(0), speed);
	while (1) {
		if (abs(target_angle - Gyro_GetAngle(0)) < 20) {
			break;
		}
		if (abs(target_angle - Gyro_GetAngle(0)) < 50) {
			Set_Wheel_Speed(speed / 2, speed / 2);
		} else {
			Set_Wheel_Speed(speed, speed);
		}
		usleep(10000);
	}
	wheel_left_direction = 0;
	wheel_right_direction = 0;

	Set_Wheel_Speed(0, 0);

	printf("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(0));
}
void Turn_Right(uint16_t speed, uint16_t angle)
{
	int16_t target_angle;
	uint16_t gyro_angle;

	gyro_angle = Gyro_GetAngle(0);

	target_angle = gyro_angle - angle;
	if (target_angle < 0) {
		target_angle = 3600 + target_angle;
	}

	wheel_left_direction = 0;
	wheel_right_direction = 1;

	Set_Wheel_Speed(speed, speed);

	printf("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(0), speed);
	while (1) {
		if (abs(target_angle - Gyro_GetAngle(0)) < 20) {
			break;
		}
		if (abs(target_angle - Gyro_GetAngle(0)) < 50) {
			Set_Wheel_Speed(speed / 2, speed / 2);
		} else {
			Set_Wheel_Speed(speed, speed);
		}
		usleep(10000);
	}
	wheel_left_direction = 0;
	wheel_right_direction = 0;

	Set_Wheel_Speed(0, 0);

	printf("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(0));
}

uint8_t Get_OBS_Status(void)
{
	uint8_t Status = 0;

	if (robot::instance()->robot_get_obs_left() > Left_OBSTrig_Value)
		Status |= Status_Left_OBS;

	if (robot::instance()->robot_get_obs_front() > Front_OBSTrig_Value)
		Status |= Status_Front_OBS;

	if (robot::instance()->robot_get_obs_right() > Right_OBSTrig_Value)
		Status |= Status_Right_OBS;

	return Status;
}


int32_t Get_FrontOBS(void)
{
	return (int32_t)robot::instance()->robot_get_obs_front();
}

int32_t Get_LeftOBS(void)
{
	return (int32_t)robot::instance()->robot_get_obs_left();
}

uint8_t Get_Bumper_Status(void)
{
	uint8_t Temp_Status = 0;

	if (robot::instance()->robot_get_bumper_left()) {
		Temp_Status |= LeftBumperTrig;
	}
	if (robot::instance()->robot_get_bumper_right()) {
		Temp_Status |= RightBumperTrig;
	}
	return Temp_Status;
}

uint8_t Get_Cliff_Trig(void)
{
	return 0;
}

uint8_t Is_AtHomeBase(void)
{
	return 0;
}

void SetHomeRemote(void)
{
}

uint8_t Is_OBS_Near(void)
{
	return 0;
}

uint32_t Get_Rcon_Status(void)
{
	return 0;
}

void Set_Rcon_Status(uint32_t code)
{
	code = code;
}

void Reset_TempPWM(void)
{
}

void Set_Wheel_Speed(uint8_t Left, uint8_t Right)
{
	int16_t left_speed, right_speed;

	left_speed = (int16_t)(Left * 7.23);
	right_speed = (int16_t)(Right * 7.23);
	if (wheel_left_direction == 1) {
		left_speed |= 0x8000;
	}
	
	if (wheel_right_direction == 1) {
		right_speed |= 0x8000;
	}

#if 0
	printf("%s %d: left: %d\tright: %d\tposition: (%f, %f, %f)\n", __FUNCTION__, __LINE__,
		left_speed, right_speed, robot::instance()->robot_get_position_x(), robot::instance()->robot_get_position_y(), robot::instance()->robot_get_position_z());
#endif
	control_set_wheel_speed(left_speed, right_speed);
}

uint8_t Check_Motor_Current(void)
{
	return 0;
}

uint8_t Self_Check(uint8_t Check_Code)
{
	Check_Code = Check_Code;
	return 0;
}

uint8_t Check_Bat_Home(void)
{
	return 0;
}

uint8_t Get_Clean_Mode(void)
{
	return 0;
}

void Set_VacMode(uint8_t data)
{
	data = data;
}

void Set_BLDC_Speed(uint32_t S)
{
	S = S;
}

void Set_Vac_Speed(void)
{
}

void OBS_Dynamic_Base(uint16_t Cy)
{
	Cy = Cy;
}

int16_t Get_FrontOBST_Value(void)
{
	return Front_OBSTrig_Value + 1700;
}

int16_t Get_LeftOBST_Value(void)
{
    return Left_OBSTrig_Value + 200;
}

uint8_t Is_WallOBS_Near(void)
{
    if (robot::instance()->robot_get_obs_front() > (Front_OBSTrig_Value + 500)) {
		return 1;
	}
    if (robot::instance()->robot_get_obs_right() > (Right_OBSTrig_Value + 500)) {
		return 1;
	}
    if (robot::instance()->robot_get_obs_left() > (Front_OBSTrig_Value + 1000)) {
		return 1;
	}
    return 0;
}

void Move_Forward(uint8_t Left_Speed, uint8_t Right_Speed)
{
	int16_t left_speed, right_speed;

	wheel_left_direction = 0;
	left_speed = (int16_t)(Left_Speed * 7.23);

	wheel_right_direction = 0;
	right_speed = (int16_t)(Right_Speed * 7.23);

	control_set_wheel_speed(left_speed, right_speed);
}

uint8_t Get_VacMode(void)
{
	return 0;
}

void Switch_VacMode(void)
{
}

uint32_t Get_Rcon_Remote(void)
{
	return 0;
}

void Reset_MoveWithRemote(void)
{
}

uint8_t Check_Bat_SetMotors(uint32_t Vacuum_Voltage, uint32_t Side_Brush, uint32_t Main_Brush)
{
	Vacuum_Voltage = Vacuum_Voltage;
	Side_Brush = Side_Brush;
	Main_Brush = Main_Brush;
	return 0;
}

void Reset_WorkTimer(void)
{
}

void Reset_Rcon_Status(void)
{
}

void Display_Battery_Status(uint8_t temp)
{
	temp = temp;
}

void Set_Dir_Left(void)
{
	wheel_left_direction = 1;
	wheel_right_direction = 0;
}

void Set_Dir_Right(void)
{
	wheel_left_direction = 0;
	wheel_right_direction = 1;
}

void Set_LED(uint16_t G, uint16_t R)
{
	control_set_led_red(R);
	control_set_led_green(G);
}

void Stop_Brifly(void)
{
	printf("%s %d: stopping robot.\n", __FUNCTION__, __LINE__);
	do {
		control_set_wheel_speed(0, 0);
		usleep(15000);
		//printf("%s %d: linear speed: (%f, %f, %f)\n", __FUNCTION__, __LINE__,
		//	robot::instance()->robot_get_linear_x(), robot::instance()->robot_get_linear_y(), robot::instance()->robot_get_linear_z());
	} while (robot::instance()->robot_is_moving());
	printf("%s %d: robot is stopped.\n", __FUNCTION__, __LINE__);
}

void Set_SideBrush_PWM(uint16_t L, uint16_t R)
{
	L = L;
	R = R;
}

uint8_t Get_LeftBrush_Stall(void)
{
	return 0;
}

uint8_t Get_RightBrush_Stall(void)
{
	return 0;
}



uint8_t Remote_Key(uint32_t Key)
{
	Key = Key;
	return 0;
}

void Reset_Touch(void)
{
}

void Set_Touch(void)
{
}

void Deceleration(void)
{
}

uint8_t Touch_Detect(void)
{
	return 0;
}

uint8_t Is_Station(void)
{
	return 0;
}

uint8_t Is_ChargerOn(void)
{
	return 0;
}

uint8_t Is_Water_Tank(void)
{
	return 0;
}

void Set_Clean_Mode(uint8_t mode)
{
	mode = mode;
}

void Beep(uint8_t Sound)
{
	Sound = Sound;
}

void Disable_Motors(void)
{
}
