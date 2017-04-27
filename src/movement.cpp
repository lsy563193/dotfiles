#include <stdint.h>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include "robot.hpp"
#include <time.h>
#include <fcntl.h>

#include "gyro.h"
#include "movement.h"
#include "crc8.h"
#include "serial.h"
#include "robotbase.h"
#include "config.h"
#include "core_move.h"
#include "wall_follow_multi.h"
#include "wav.h"
extern uint8_t sendStream[SEND_LEN];

static int16_t Left_OBSTrig_Value = 500;
static int16_t Front_OBSTrig_Value = 500;
static int16_t Right_OBSTrig_Value = 500;
volatile int16_t OBS_Trig_Value = 800;
static int16_t Leftwall_OBSTrig_Vale = 500;
static uint8_t wheel_left_direction = 0;
static uint8_t wheel_right_direction = 0;
static uint8_t remote_move_flag=0;
static uint8_t home_remote_flag = 0;
static uint8_t Gyro_Status = 0;
uint32_t Rcon_Status;

uint32_t Average_Move = 0;
uint32_t Average_Counter =0;
uint32_t Max_Move = 0;
uint32_t Auto_Work_Time = 2800;
uint32_t Room_Work_Time = 3600;
uint8_t Room_Mode = 0;

static uint32_t Wall_Accelerate =0;
static int16_t Left_Wheel_Speed = 0;
static int16_t Right_Wheel_Speed = 0;
static uint32_t left_wheel_step = 0;
static uint32_t right_wheel_step = 0;
static uint32_t leftwall_step = 0;
static uint32_t rightwall_step = 0;

//Value for saving SideBrush_PWM
static uint16_t LBrush_PWM = 0;
static uint16_t RBrush_PWM = 0;

static int32_t MoveStepCounter = 0;
static uint32_t Mobility_Step = 0;
static uint8_t Direction_Flag=0;
// Variable for vacuum mode

volatile uint8_t Vac_Mode;
static uint8_t Cleaning_mode = 0;
static uint8_t sendflag=0;
static time_t work_time;
ros::Time lw_t,rw_t; // these variable is used for calculate wheel step

// Flag for homeremote
volatile uint8_t R_H_Flag=0;

// Counter for bumper error
volatile uint8_t Bumper_Error = 0;

// Value for wall sensor offset.
volatile int16_t Left_Wall_BaseLine = 50;
volatile int16_t Right_Wall_BaseLine = 50;

// Variable for key status, key may have many key types.
volatile uint8_t Key_Status = 0;
// Variable for touch status, touch status is just for KEY_CLEAN.
volatile uint8_t Touch_Status = 0;
// Variable for remote status, remote status is just for remote controller.
volatile uint8_t Remote_Status = 0;
// Variable for plan status
volatile bool Plan_Status = false;

// Error code for exception case
volatile uint8_t Error_Code = 0;
/*----------------------- Work Timer functions--------------------------*/
void Reset_Work_Time()
{
	work_time = time(NULL);
}

uint32_t Get_Work_Time()
{
	return (uint32_t)difftime(time(NULL), work_time);
}

void Set_Work_Time(time_t time)
{
	work_time = time;
}

/*----------------------- Set error functions--------------------------*/
void Set_Error_Code(uint8_t code)
{
	Error_Code = code;
}

uint8_t Get_Error_Code()
{
	return Error_Code;
}

void Set_LeftBrush_Stall(uint8_t L)
{
	// Actually not used in old code
	L = L;
}

uint32_t Get_RightWheel_Step(void)
{
	double t,step;
	double rwsp;
	if(Right_Wheel_Speed<0)
		rwsp = (double)Right_Wheel_Speed*-1;
	else
		rwsp = (double)Right_Wheel_Speed;
	t = (double)(ros::Time::now()-rw_t).toSec();
	step = rwsp*t/0.12;//origin 0.181
	right_wheel_step = (uint32_t)step;
	return right_wheel_step;
}
uint32_t Get_LeftWheel_Step(void)
{
	double t,step;
	double lwsp;
	if(Left_Wheel_Speed<0)
		lwsp = (double)Left_Wheel_Speed*-1;
	else
		lwsp = (double)Left_Wheel_Speed;
	t=(double)(ros::Time::now()-lw_t).toSec();
	step = lwsp*t/0.12;//origin 0.181
	left_wheel_step = (uint32_t)step;
	return left_wheel_step;
}

void Reset_Wheel_Step(void)
{
	lw_t = ros::Time::now();
	rw_t = ros::Time::now();
	right_wheel_step = 0;
	left_wheel_step = 0;
}

void Reset_Wall_Step(void)
{
	leftwall_step = 0;
	rightwall_step = 0;
}

uint32_t Get_LeftWall_Step(void)
{
	return leftwall_step = Get_LeftWheel_Step();
}

uint32_t Get_RightWall_Step(void)
{
	return rightwall_step = Get_RightWheel_Step();
}

void Set_Wheel_Step(uint32_t Left, uint32_t Right)
{
	left_wheel_step = Left;
	right_wheel_step = Right;
}

int32_t Get_Wall_ADC(int8_t dir)
{
	if (dir == 0)
	{
		return (int32_t)(int16_t)robot::instance()->robot_get_left_wall();
	}
	else
	{
		return (int32_t)(int16_t)robot::instance()->robot_get_right_wall();
	}
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
	//ROS_INFO("Run Wall_Dynamic_Base.");
	static int32_t Left_Wall_Sum_Value=0, Right_Wall_Sum_Value=0;
	static int32_t Left_Wall_Everage_Value=0, Right_Wall_Everage_Value=0;
	static int32_t Left_Wall_E_Counter=0, Right_Wall_E_Counter=0;
	static int32_t Left_Temp_Wall_Buffer=0, Right_Temp_Wall_Buffer=0;

	// Dynamic adjust for left wall sensor.
	Left_Temp_Wall_Buffer = Get_Wall_ADC(0);
	Left_Wall_Sum_Value+=Left_Temp_Wall_Buffer;
	Left_Wall_E_Counter++;
	Left_Wall_Everage_Value=Left_Wall_Sum_Value/Left_Wall_E_Counter;

	if(ABS_Minus(Left_Wall_Everage_Value,Left_Temp_Wall_Buffer)>20)
	{
		Left_Wall_Everage_Value=0;
		Left_Wall_E_Counter=0;
		Left_Wall_Sum_Value=0;
		Left_Temp_Wall_Buffer=0;
	}
	if ((uint32_t) Left_Wall_E_Counter > Cy)
	{
		// Get the wall base line for left wall sensor.
		Left_Wall_Everage_Value += Get_Wall_Base(0);
		if(Left_Wall_Everage_Value>300)Left_Wall_Everage_Value=300;//set a limit
		// Adjust the wall base line for left wall sensor.
		Set_Wall_Base(0, Left_Wall_Everage_Value);
		Left_Wall_Everage_Value=0;
		Left_Wall_E_Counter=0;
		Left_Wall_Sum_Value=0;
		Left_Temp_Wall_Buffer=0;
		//Beep(1, 10, 0, 1);
		//ROS_INFO("Set Left Wall base value as: %d.", Get_Wall_Base(0));
	}

	// Dynamic adjust for right wall sensor.
	Right_Temp_Wall_Buffer = Get_Wall_ADC(1);
	Right_Wall_Sum_Value+=Right_Temp_Wall_Buffer;
	Right_Wall_E_Counter++;
	Right_Wall_Everage_Value=Right_Wall_Sum_Value/Right_Wall_E_Counter;

	if(ABS_Minus(Right_Wall_Everage_Value,Right_Temp_Wall_Buffer)>20)
	{
		Right_Wall_Everage_Value=0;
		Right_Wall_E_Counter=0;
		Right_Wall_Sum_Value=0;
		Right_Temp_Wall_Buffer=0;
	}
	if ((uint32_t) Right_Wall_E_Counter > Cy)
	{
		// Get the wall base line for right wall sensor.
		Right_Wall_Everage_Value += Get_Wall_Base(1);
		if(Right_Wall_Everage_Value>300)Right_Wall_Everage_Value=300;//set a limit
		// Adjust the wall base line for right wall sensor.
		Set_Wall_Base(1, Right_Wall_Everage_Value);
		Right_Wall_Everage_Value=0;
		Right_Wall_E_Counter=0;
		Right_Wall_Sum_Value=0;
		Right_Temp_Wall_Buffer=0;
		//Beep(1, 10, 0, 1);
		//ROS_INFO("Set Right Wall base value as: %d.", Get_Wall_Base(0));
	}

}

void Set_Wall_Base(int8_t dir, int32_t data)
{
	if (dir == 0)
	{
		Left_Wall_BaseLine = data;
	}
	else
	{
		Right_Wall_BaseLine = data;
	}
}
int32_t Get_Wall_Base(int8_t dir)
{
	if(dir == 0)
	{
		return Left_Wall_BaseLine;
	}
	else
	{
		return Right_Wall_BaseLine;
	}
}

void Quick_Back(uint8_t Speed, uint16_t Distance)
{
	// Quickly move back for a distance.
	wheel_left_direction = 1;
	wheel_right_direction = 1;
	Reset_Wheel_Step();
	Set_Wheel_Speed(Speed, Speed);
	// This count is for how many milliseconds it should take. The Distance is in mm.
	int back_count = int(1000 * Distance / (Speed * SPEED_ALF));
	//ROS_INFO("%s %d Quick_back for %dms.", __FUNCTION__, __LINE__, back_count);
	for (int i = 0; i < back_count; i++){
		// Sleep for 1 millisecond
		usleep(1000);
		if (Touch_Detect())
		{
			break;
		}
		if (Remote_Key(Remote_Clean))
		{
			Reset_Rcon_Remote();
			break;
		}
	}
	ROS_INFO("Quick_Back finished.");
}

void Turn_Left_At_Init(uint16_t speed, int16_t angle)
{
	int16_t target_angle;
	int16_t gyro_angle;

	gyro_angle = Gyro_GetAngle();

	target_angle = gyro_angle + angle;
	if (target_angle >= 3600) {
		target_angle -= 3600;
	}
	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);

	Set_Dir_Left();

	Set_Wheel_Speed(speed, speed);

	uint8_t oc=0;
	uint8_t accurate;
	accurate = 10;
	if(speed > 30) accurate  = 30;
	while (ros::ok()) {
		if (abs(target_angle - Gyro_GetAngle()) < accurate) {
			break;
		}
		if (abs(target_angle - Gyro_GetAngle()) < 50) {
			auto speed_ = std::min((uint16_t)5,speed);
			Set_Wheel_Speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), 5);
		}
		else if (abs(target_angle - Gyro_GetAngle()) < 200) {
			auto speed_ = std::min((uint16_t)5,speed);
			Set_Wheel_Speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), 10);
		}
		else {
			Set_Wheel_Speed(speed, speed);
		}
		oc= Check_Motor_Current();
		if(oc == Check_Left_Wheel || oc== Check_Right_Wheel)
			break;
		if(Touch_Detect())
			break;
		if(Is_Turn_Remote())
			break;
//		if(Get_Bumper_Status()){
//			Stop_Brifly();
//			WFM_move_back(120);
//			Stop_Brifly();
//			Set_Dir_Left();
//			ROS_INFO("Bumper triged when turn left, back 20mm.");
//		}
		usleep(10000);
		//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d, diff = %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed,target_angle - Gyro_GetAngle());
	}
	wheel_left_direction = 0;
	wheel_right_direction = 0;

	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle());
}

void Turn_Left(uint16_t speed, int16_t angle)
{
	int16_t target_angle;
	int16_t gyro_angle;

	gyro_angle = Gyro_GetAngle();

	target_angle = gyro_angle + angle;
	if (target_angle >= 3600) {
		target_angle -= 3600;
	}
	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);

	Set_Dir_Left();

	Set_Wheel_Speed(speed, speed);

	uint8_t oc=0;
	uint8_t accurate;
	accurate = 10;
	if(speed > 30) accurate  = 30;
	while (ros::ok()) {
		if (abs(target_angle - Gyro_GetAngle()) < accurate) {
			break;
		}
		if (abs(target_angle - Gyro_GetAngle()) < 50) {
			auto speed_ = std::min((uint16_t)5,speed);
			Set_Wheel_Speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), 5);
		}
		else if (abs(target_angle - Gyro_GetAngle()) < 200) {
			auto speed_ = std::min((uint16_t)5,speed);
			Set_Wheel_Speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), 10);
		}
		else {
			Set_Wheel_Speed(speed, speed);
		}
		oc= Check_Motor_Current();
		if(oc == Check_Left_Wheel || oc== Check_Right_Wheel)
			break;
		if(Touch_Detect())
			break;
		/*if(Is_Turn_Remote())
			break;*/
		if(Get_Bumper_Status()){
			Stop_Brifly();
			WFM_move_back(120);
			Stop_Brifly();
			Set_Dir_Left();
			ROS_INFO("Bumper triged when turn left, back 20mm.");
		}
		usleep(10000);
		//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d,diff = %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed,target_angle - Gyro_GetAngle());
	}
	wheel_left_direction = 0;
	wheel_right_direction = 0;

	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle());
}

void Turn_Right(uint16_t speed, int16_t angle)
{
	int16_t target_angle;
	int16_t gyro_angle;

	gyro_angle = Gyro_GetAngle();

	target_angle = gyro_angle - angle;
	if (target_angle < 0) {
		target_angle = 3600 + target_angle;
	}
	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);

	Set_Dir_Right();

	Set_Wheel_Speed(speed, speed);
	uint8_t oc=0;

	uint8_t accurate;
	accurate = 10;
	if(speed > 30) accurate  = 30;
	while (ros::ok()) {
		if (abs(target_angle - Gyro_GetAngle()) < accurate) {
			break;
		}
		if (abs(target_angle - Gyro_GetAngle()) < 50) {
			auto speed_ = std::min((uint16_t)5,speed);
			Set_Wheel_Speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), 5);
		}
		else if (abs(target_angle - Gyro_GetAngle()) < 200) {
			auto speed_ = std::min((uint16_t)5,speed);
			Set_Wheel_Speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), 10);
		}
		else {
			Set_Wheel_Speed(speed, speed);
		}
		oc= Check_Motor_Current();
		if(oc == Check_Left_Wheel || oc== Check_Right_Wheel)
			break;
		if(Touch_Detect())
			break;
		/*if(Is_Turn_Remote())
			break;*/
		if(Get_Bumper_Status()){
			Stop_Brifly();
			WFM_move_back(120);
			Stop_Brifly();
			Set_Dir_Right();
			ROS_INFO("Bumper triged when turn right, back 20mm.");
		}
		usleep(10000);
		//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);
	}
	wheel_left_direction = 0;
	wheel_right_direction = 0;

	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle());
}

void WF_Turn_Right(uint16_t speed, int16_t angle)
{
	int16_t target_angle;
	int16_t gyro_angle;
	int32_t i, j;
	float pos_x, pos_y;
	gyro_angle = Gyro_GetAngle();

	target_angle = gyro_angle - angle;
	if (target_angle < 0) {
		target_angle = 3600 + target_angle;
	}
	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);

	Set_Dir_Right();

	Set_Wheel_Speed(speed, speed);
	uint8_t oc=0;

	uint8_t accurate;
	accurate = 10;
	if(speed > 30) accurate  = 30;
	while (ros::ok()) {
		pos_x = robot::instance()->robot_get_position_x() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
		pos_y = robot::instance()->robot_get_position_y() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
		Map_SetPosition(pos_x, pos_y);

		i = Map_GetRelativeX(Gyro_GetAngle(), CELL_SIZE_3, 0);
		j = Map_GetRelativeY(Gyro_GetAngle(), CELL_SIZE_3, 0);
		if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BOUNDARY) {
			Map_SetCell(MAP, i, j, BLOCKED_OBS);
		}

		if (abs(target_angle - Gyro_GetAngle()) < accurate) {
			break;
		}
		if (abs(target_angle - Gyro_GetAngle()) < 50) {
			auto speed_ = std::min((uint16_t)5,speed);
			Set_Wheel_Speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), 5);
		}
		else if (abs(target_angle - Gyro_GetAngle()) < 200) {
			auto speed_ = std::min((uint16_t)5,speed);
			Set_Wheel_Speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), 10);
		}
		else {
			Set_Wheel_Speed(speed, speed);
		}
		oc= Check_Motor_Current();
		if(oc == Check_Left_Wheel || oc== Check_Right_Wheel)
			break;
		if(Touch_Detect())
			break;
		/*if(Is_Turn_Remote())
			break;
		 */

		if (Get_Rcon_Remote() & (Remote_Home | Remote_Clean))
		{
			break;
		}

		if(Get_Bumper_Status()){
			Stop_Brifly();
			WFM_move_back(120);
			Stop_Brifly();
			Set_Dir_Right();
			ROS_INFO("Bumper triged when turn right, back 20mm.");
		}
		usleep(10000);
		//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);
	}
	wheel_left_direction = 0;
	wheel_right_direction = 0;

	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle());
}

void Jam_Turn_Left(uint16_t speed, int16_t angle)
{
	int16_t target_angle;
	int16_t gyro_angle;

	gyro_angle = Gyro_GetAngle();

	target_angle = gyro_angle + angle;
	if (target_angle >= 3600) {
		target_angle -= 3600;
	}
	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);

	Set_Dir_Left();

	Set_Wheel_Speed(speed, speed);

	uint8_t oc=0;
	uint8_t accurate;
	accurate = 10;
	if(speed > 30) accurate  = 30;
	while (ros::ok()) {
		if (abs(target_angle - Gyro_GetAngle()) < accurate) {
			break;
		}
		if (abs(target_angle - Gyro_GetAngle()) < 50) {
			auto speed_ = std::min((uint16_t)5,speed);
			Set_Wheel_Speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), 5);
		}
		else if (abs(target_angle - Gyro_GetAngle()) < 200) {
			auto speed_ = std::min((uint16_t)5,speed);
			Set_Wheel_Speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), 10);
		}
		else {
			Set_Wheel_Speed(speed, speed);
		}
		oc= Check_Motor_Current();
		if(oc == Check_Left_Wheel || oc== Check_Right_Wheel)
			break;
		if(Touch_Detect())
			break;
		/*if(Is_Turn_Remote())
			break;*/
		usleep(10000);
		//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d,diff = %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed,target_angle - Gyro_GetAngle());
	}
	wheel_left_direction = 0;
	wheel_right_direction = 0;

	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle());
}

void Jam_Turn_Right(uint16_t speed, int16_t angle)
{
	int16_t target_angle;
	int16_t gyro_angle;

	gyro_angle = Gyro_GetAngle();

	target_angle = gyro_angle - angle;
	if (target_angle < 0) {
		target_angle = 3600 + target_angle;
	}
	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);

	Set_Dir_Right();

	Set_Wheel_Speed(speed, speed);
	uint8_t oc=0;

	uint8_t accurate;
	accurate = 10;
	if(speed > 30) accurate  = 30;
	while (ros::ok()) {
		if (abs(target_angle - Gyro_GetAngle()) < accurate) {
			break;
		}
		if (abs(target_angle - Gyro_GetAngle()) < 50) {
			auto speed_ = std::min((uint16_t)5,speed);
			Set_Wheel_Speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), 5);
		}
		else if (abs(target_angle - Gyro_GetAngle()) < 200) {
			auto speed_ = std::min((uint16_t)5,speed);
			Set_Wheel_Speed(speed_, speed_);
			//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), 10);
		}
		else {
			Set_Wheel_Speed(speed, speed);
		}
		oc= Check_Motor_Current();
		if(oc == Check_Left_Wheel || oc== Check_Right_Wheel)
			break;
		if(Touch_Detect())
			break;
		/*if(Is_Turn_Remote())
			break;*/
		usleep(10000);
		//ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);
	}
	wheel_left_direction = 0;
	wheel_right_direction = 0;

	Set_Wheel_Speed(0, 0);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle());
}
int32_t Get_FrontOBS(void)
{
	return (int32_t)robot::instance()->robot_get_obs_front();
}

int32_t Get_LeftOBS(void)
{
	return (int32_t)robot::instance()->robot_get_obs_left();
}
int32_t Get_RightOBS(void)
{
	return (int32_t)robot::instance()->robot_get_obs_right();
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
	uint8_t Cliff_Status = 0x00;
	int16_t cl,cr,cf;
	cl = robot::instance()->robot_get_cliff_left();
	cr = robot::instance()->robot_get_cliff_right();
	cf = robot::instance()->robot_get_cliff_front();	
	if (cl < Cliff_Limit){
		ROS_WARN("Left cliff is detected:%d", cl);
		Cliff_Status += 0x01;
	}
	if (cr< Cliff_Limit){
		ROS_WARN("Right cliff is detected:%d", cr);
		Cliff_Status += 0x02;
	}
	if (cf < Cliff_Limit){
		ROS_WARN("Front cliff is detected:%d", cf);
		Cliff_Status += 0x04;
	}
	/*
	if (Cliff_Status != 0x00){
		ROS_WARN("Return Cliff status:%x.", Cliff_Status);
	}
	*/
	return Cliff_Status;
}

uint8_t Cliff_Escape(void)
{
	uint8_t count=1;
	uint8_t cc;	
	while(ros::ok()){
		cc = Get_Cliff_Trig();
		if(cc){
			if(cc==(Status_Cliff_Left|Status_Cliff_Right|Status_Cliff_Front)){
				return 1;
			}
			switch(count++){
				case 1:
						Cliff_Turn_Right(30,300);
						break;
				case 2:
						Cliff_Turn_Left(30,300);
						break;
				case 3:
						Cliff_Turn_Right(30,300);
						break;
				case 4:
						Cliff_Turn_Left(30,300);
						break;
				case 5:
						Move_Back();
						Cliff_Turn_Left(30,800);
						break;
				default:
					return 1;
			}
			
		}
		else
			return 0;
	}
	return 0;
}

bool except_event()
{
//	uint8_t oc = Check_Motor_Current();
//		if(oc == Check_Left_Wheel || oc== Check_Right_Wheel)
//			return true;
	if(Touch_Detect())
	{
		ROS_WARN("Touch_Detect in except_event!");
		return true;
	}
//		if(Is_Turn_Remote())
//			return true;
	return false;
}

uint8_t Cliff_Event(uint8_t event)
{
	uint16_t temp_adjust=0,random_factor=0;
	uint8_t d_flag = 0;
	// There is 50% chance that the temp_adjust = 450.
	//if(left_wheel_step%2)temp_adjust = 450;
	if(((int)ros::Time::now().toSec())%2)temp_adjust = 450;
	else temp_adjust = 0;
	// There is 33% chance that the random_factor = 1.
	//if(right_wheel_step%3)random_factor = 1;
	if(((int)ros::Time::now().toSec())%3)random_factor = 1;
	else random_factor = 0;

	switch(event){
		case Status_Cliff_Left:
			Cliff_Turn_Right(Turn_Speed,temp_adjust+900);
			break;
		case Status_Cliff_Right:
			Cliff_Turn_Left(Turn_Speed,temp_adjust+900);
			d_flag = 1;
			break;
		case Status_Cliff_Front:
			if(random_factor)
			{
				Cliff_Turn_Left(Turn_Speed,1200+temp_adjust);
				d_flag = 1;
			}
			else Cliff_Turn_Right(Turn_Speed,1300+temp_adjust);
			break;
		case (Status_Cliff_Left|Status_Cliff_Front):
			Cliff_Turn_Right(Turn_Speed,1650+temp_adjust);
			break;
		case (Status_Cliff_Right|Status_Cliff_Front):
			Cliff_Turn_Left(Turn_Speed,1650+temp_adjust);
			d_flag = 1;
			break;
		case (Status_Cliff_Left|Status_Cliff_Front|Status_Cliff_Right):
			Cliff_Turn_Right(Turn_Speed,1700);
			break;
		case 0:
			break;
		default:
			Cliff_Turn_Left(Turn_Speed,1800);
			d_flag = 1;
			break;
	}
	Move_Forward(30,30);
	if(d_flag)return 1;
	return 0;
}
/*-------------------------------Check if at home base------------------------------------*/
uint8_t Is_AtHomeBase(void)
{
	// If the charge status is true, it means it is at home base charging.
	//Debug
	if (robot::instance()->robot_get_charge_status() == 2 || robot::instance()->robot_get_charge_status() == 1){
		return 1;
	}else{
		return 0;
	}
}

uint8_t Turn_Connect(void)
{
	// This function is for trying turning left and right to adjust the pose of robot, so that it can charge.

	int16_t target_angle;
	int8_t speed = 5;
	// Enable the switch for charging.
	set_start_charge();
	// Wait for 200ms for charging activated.
	usleep(200000);
	if (Is_ChargerOn())
	{
		ROS_INFO("[movement.cpp] Reach charger without turning.");
//		Beep(2, 25, 0, 1);
		return 1;
	}
	// Start turning right.
	target_angle = Gyro_GetAngle() - 120;
	if (target_angle < 0) {
		target_angle = 3600 + target_angle;
	}
	wheel_left_direction = 0;
	wheel_right_direction = 1;
	Set_Wheel_Speed(speed, speed);
	while(abs(target_angle - Gyro_GetAngle()) > 20)
	{
		if(Is_ChargerOn())
		{
			Disable_Motors();
			Stop_Brifly();
			// Wait for a while to decide if it is really on the charger stub.
			usleep(500000);
			if(Is_ChargerOn())
			{
				ROS_INFO("[movement.cpp] Turn left reach charger.");
//				Beep(2, 25, 0, 1);
				return 1;
			}
			Set_Wheel_Speed(speed, speed);
		}
		if(Touch_Detect())
		{
			ROS_INFO("%s %d: Touch_Detect.", __FUNCTION__, __LINE__);
			Disable_Motors();
			return 0;
		}
	}
	Stop_Brifly();
	// Start turning left.
	target_angle = Gyro_GetAngle() + 240;
	if (target_angle > 3600) {
		target_angle = target_angle - 3600;
	}
	wheel_left_direction = 1;
	wheel_right_direction = 0;
	Set_Wheel_Speed(speed, speed);
	while(abs(target_angle - Gyro_GetAngle()) > 20)
	{
		if(Is_ChargerOn())
		{
			Disable_Motors();
			Stop_Brifly();
			if(Is_ChargerOn())
			{
				ROS_INFO("[movement.cpp] Turn right reach charger.");
//				Beep(2, 25, 0, 1);
				return 1;
			}
			Set_Wheel_Speed(speed, speed);
		}
		if(Touch_Detect())
		{
			ROS_INFO("%s %d: Touch_Detect.", __FUNCTION__, __LINE__);
			Disable_Motors();
			return 0;
		}
	}
	Stop_Brifly();

	return 0;
}

void SetHomeRemote(void)
{
	home_remote_flag = 1;
}

void Reset_HomeRemote(void)
{
	home_remote_flag = 0;
}

uint8_t IsHomeRemote(void)
{
	return R_H_Flag;
}

uint8_t  Is_MoveWithRemote(void){
	return remote_move_flag;	
}

uint8_t Is_OBS_Near(void)
{
	if(robot::instance()->robot_get_obs_front() > (Front_OBSTrig_Value-200))return 1;
	if(robot::instance()->robot_get_obs_right() > (Right_OBSTrig_Value-200))return 1;
	if(robot::instance()->robot_get_obs_left() > (Left_OBSTrig_Value-200))return 1;
	return 0;
}

void Reset_TempPWM(void)
{
}

void Set_Wheel_Speed(uint8_t Left, uint8_t Right)
{
	//ROS_INFO("Set wheel speed:%d, %d.", Left, Right);
	Left = Left < RUN_TOP_SPEED ? Left : RUN_TOP_SPEED;
	Right = Right < RUN_TOP_SPEED ? Right : RUN_TOP_SPEED;
	Set_LeftWheel_Speed(Left);
	Set_RightWheel_Speed(Right);

#if GYRO_DYNAMIC_ADJUSTMENT
	// Add handling for gyro dynamic adjustment.
	// If robot going straight, should turn off gyro dynamic adjustment.
	// If robot turning, should turn on gyro dynamic adjustment.
	if (abs(Left - Right) > 1)
	{
		Set_Gyro_Dynamic_On();
	}
	else
	{
		Set_Gyro_Dynamic_Off();
	}
#endif
}

void Set_LeftWheel_Speed(uint8_t speed)
{
	int16_t l_speed;
	speed = speed>RUN_TOP_SPEED?RUN_TOP_SPEED:speed;
	l_speed = (int16_t)(speed*SPEED_ALF);
	if(wheel_left_direction == 1)
		l_speed |=0x8000;
	Left_Wheel_Speed = l_speed;
	control_set(CTL_WHEEL_LEFT_HIGH, (l_speed >> 8) & 0xff);
	control_set(CTL_WHEEL_LEFT_LOW, l_speed & 0xff);

}

void Set_RightWheel_Speed(uint8_t speed)
{
	int16_t r_speed;
	speed = speed>RUN_TOP_SPEED?RUN_TOP_SPEED:speed;
	r_speed = (int16_t)(speed*SPEED_ALF);
	if(wheel_right_direction == 1)
		r_speed |=0x8000;
	Right_Wheel_Speed = r_speed;
	control_set(CTL_WHEEL_RIGHT_HIGH, (r_speed >> 8) & 0xff);
	control_set(CTL_WHEEL_RIGHT_LOW, r_speed & 0xff);
}

int16_t Get_LeftWheel_Speed(void)
{
	return Left_Wheel_Speed;
}

int16_t Get_RightWheel_Speed(void)
{
	return Right_Wheel_Speed;
}

void Work_Motor_Configure(void)
{
	// Set the vacuum to a normal mode
	Set_VacMode(Vac_Normal);
	Set_Vac_Speed();

	// Trun on the main brush and side brush
	Set_SideBrush_PWM(50, 50);
	Set_MainBrush_PWM(30);
}

uint8_t Check_Motor_Current(void)
{
	static uint8_t lwheel_oc_count = 0;
	static uint8_t rwheel_oc_count = 0;
	if((uint32_t)robot::instance()->robot_get_lwheel_current() > Wheel_Stall_Limit){
		lwheel_oc_count++;
		if(lwheel_oc_count >40){
			lwheel_oc_count =0;
			ROS_WARN("%s,%d,left wheel over current,%lu mA\n",__FUNCTION__,__LINE__,(uint32_t)robot::instance()->robot_get_lwheel_current());
			return Check_Left_Wheel;
		}
	}
	else
		lwheel_oc_count = 0;
	if((uint32_t)robot::instance()->robot_get_rwheel_current() > Wheel_Stall_Limit){
		rwheel_oc_count++;
		if(rwheel_oc_count > 40){
			rwheel_oc_count = 0;
			ROS_WARN("%s,%d,right wheel over current,%lu mA\n",__FUNCTION__,__LINE__,(uint32_t)robot::instance()->robot_get_rwheel_current());
			return Check_Right_Wheel;
		}
	}
	else
		rwheel_oc_count = 0;
	Check_SideBrush_Stall();
	if(robot::instance()->robot_get_rbrush_oc()){
		ROS_WARN("%s,%d,right brush over current\n",__FUNCTION__,__LINE__);
		return Check_Right_Brush;
	}
	if(robot::instance()->robot_get_lbrush_oc()){
		ROS_WARN("%s,%d,left brush over current\n",__FUNCTION__,__LINE__);
		return Check_Left_Brush;
	}
	if(robot::instance()->robot_get_mbrush_oc()){
		ROS_WARN("%s,%d,main brush over current\n",__FUNCTION__,__LINE__);
		return Check_Main_Brush;
	}
	if(robot::instance()->robot_get_vacuum_oc()){
		ROS_WARN("%s,%d,vacuum over current\n",__FUNCTION__,__LINE__);
		return Check_Vacuum;	
	}
	return 0;
}

/*-----------------------------------------------------------Self Check-------------------*/
uint8_t Self_Check(uint8_t Check_Code)
{
////	static uint8_t
////	uint32_t Temp_Brush_Current=0;
////	uint8_t Temp_Brush_Current_Count=0;
	uint8_t Time_Out=0;
	int32_t Wheel_Current_Summary=0;
	uint8_t Left_Wheel_Slow=0;
	uint8_t Right_Wheel_Slow=0;

	if(Get_Clean_Mode() == Clean_Mode_Navigation)
		CM_CorBack(COR_BACK_20MM);
	else
		Quick_Back(30,20);
	Disable_Motors();
	usleep(100000);
	/*------------------------------Self Check right wheel -------------------*/
	if(Check_Code==Check_Right_Wheel)
	{
		Right_Wheel_Slow=0;
		if(Get_Direction_Flag() == Direction_Flag_Left)
		{
			Set_Dir_Right();
		}
		else
		{
			Set_Dir_Left();
		}
		Set_Wheel_Speed(30,30);
		usleep(50000);
		Time_Out=4;
		Wheel_Current_Summary=0;
		while(Time_Out--)
		{
			Wheel_Current_Summary += robot::instance()->robot_get_rwheel_current();
			usleep(20000);
		}
		Wheel_Current_Summary/=4;
		if(Wheel_Current_Summary>Wheel_Stall_Limit)
		{
			Disable_Motors();
			wav_play(WAV_ERROR_RIGHT_WHEEL);
			ROS_WARN("%s,%d right wheel stall maybe, please check!!\n",__FUNCTION__,__LINE__);
			Set_Error_Code(Error_Code_RightWheel);
			return 1;

		}
		if(Right_Wheel_Slow>100)
		{
			Disable_Motors();
			Set_Error_Code(Error_Code_RightWheel);
			return 1;
		}
		Stop_Brifly();
		Turn_Right(Turn_Speed,1800);
	}
	/*---------------------------Self Check left wheel -------------------*/
	else if(Check_Code==Check_Left_Wheel)
	{
		Left_Wheel_Slow=0;
		if(Get_Direction_Flag() == Direction_Flag_Right)
		{
			Set_Dir_Left();
		}
		else
		{
			Set_Dir_Right();
		}
		Set_Wheel_Speed(30,30);
		usleep(50000);
		Time_Out=4;
		Wheel_Current_Summary=0;
		while(Time_Out--)
		{
			Wheel_Current_Summary += robot::instance()->robot_get_lwheel_current();
			usleep(20000);
		}
		Wheel_Current_Summary/=4;
		if(Wheel_Current_Summary>Wheel_Stall_Limit)
		{
			Disable_Motors();
			wav_play(WAV_ERROR_LEFT_WHEEL);
			ROS_WARN("%s %d,left wheel stall maybe, please check!!\n");
			Set_Error_Code(Error_Code_RightWheel);
			return 1;
		}
		if(Left_Wheel_Slow>100)
		{
			Disable_Motors();
			Set_Error_Code(Error_Code_RightWheel);
			return 1;
		}
		Stop_Brifly();
		Turn_Left(Turn_Speed,1800);
	}
	else if(Check_Code==Check_Main_Brush)
	{
		CM_CorBack(COR_BACK_20MM);
		Turn_Right(Turn_Speed,1800);
		Set_MainBrush_PWM(60);
		usleep(100000);
		//if(GPIOD->IDR&MCU_MAINBRUSH_I_DET)
		//{
			Set_Error_Code(Error_Code_MainBrush);
			Disable_Motors();
			wav_play(WAV_ERROR_MAIN_BRUSH);
			return 1;
		//}
		//Reset_MainStall();
	}
	else if(Check_Code==Check_Vacuum)
	{
		#ifdef BLDC_INSTALL
		BLDC_OFF;
		usleep(10000);
		Set_BLDC_TPWM(30);
		Set_Vac_Speed();
		BLDC_ON;
		usleep(100000);
		//if(GPIOD->IDR&MCU_VACUUM_I_DET)
		//{
			Set_Error_Code(Error_Code_Fan_H);
			Disable_Motors();
			return 1;
		//}
		#else
		Set_BLDC_Speed(80);
		sleep(1);
		//if(GPIOD->IDR&MCU_VACUUM_I_DET)
		//{
			Set_Error_Code(Error_Code_Fan_H);
			Disable_Motors();
			wav_play(WAV_ERROR_SUCTION_FAN);
			return 1;
		//}
		#endif
	}
	Stop_Brifly();
	Left_Wheel_Slow=0;
	Right_Wheel_Slow=0;
	Work_Motor_Configure();
	Move_Forward(5,5);
	return 0;
}

uint8_t Check_Bat_Home(void)
{
	// Check if battary is lower than the low battery go home voltage value.
	if (GetBatteryVoltage() > 0 && GetBatteryVoltage() < LOW_BATTERY_GO_HOME_VOLTAGE){
		return 1;
	}
	return 0;
}

uint8_t Check_Bat_Full(void)
{
	// Check if battary is higher than the battery full voltage value.
	if (GetBatteryVoltage() > BATTERY_FULL_VOLTAGE){
		return 1;
	}
	return 0;
}

uint8_t Check_Bat_Ready_To_Clean(void)
{
	uint16_t battery_limit;
	if (Get_Clean_Mode() == Clean_Mode_Charging)
	{
		battery_limit = BATTERY_READY_TO_CLEAN_VOLTAGE + 60;
	}
	else
	{
		battery_limit = BATTERY_READY_TO_CLEAN_VOLTAGE;
	}
	//ROS_INFO("%s %d: Battery limit is %d.", __FUNCTION__, __LINE__, battery_limit);
	// Check if battary is lower than the low battery go home voltage value.
	if (GetBatteryVoltage() > battery_limit){
		return 1;
	}
	return 0;
}

uint8_t Get_Clean_Mode(void)
{
	return Cleaning_mode;
}

void Set_VacMode(uint8_t data)
{
	// Set the mode for vacuum.
	// The data should be Vac_Speed_Max/Vac_Speed_Normal/Vac_Speed_NormalL.
	Vac_Mode = data;
}

void Set_BLDC_Speed(uint32_t S)
{
	// Set the power of BLDC, S should be in range(0, 100).
	S = S < 100 ? S : 100;
	control_set(CTL_VACCUM_PWR, S & 0xff);
}

void Set_Vac_Speed(void)
{
	// Set the power of BLDC according to different situation
	// Stop the BLDC if rGobot carries the water tank
	if (Is_Water_Tank()){
		Set_BLDC_Speed(0);
	}else{
		// Set the BLDC power to max if robot in max mode
		if (Get_VacMode() == Vac_Max){
			Set_BLDC_Speed(Vac_Speed_Max);
		}else{
			// If work time less than 2 hours, the BLDC should be in normal level, but if more than 2 hours, it should slow down a little bit.
			if (Get_Work_Time() < Two_Hours){
				Set_BLDC_Speed(Vac_Speed_Normal);
			}else{
				//ROS_INFO("%s %d: Work time more than 2 hours.", __FUNCTION__, __LINE__);
				Set_BLDC_Speed(Vac_Speed_NormalL);
			}
		}
	}
}

/*--------------------------------------Obs Dynamic adjust----------------------*/
void OBS_Dynamic_Base(uint16_t Cy)
{
	static int32_t Front_OBS_Buffer = 0, Left_OBS_Buffer = 0, Right_OBS_Buffer = 0;
	static uint32_t FrontOBS_E_Counter = 0,LeftOBS_E_Counter = 0,RightOBS_E_Counter = 0;
	static int32_t FrontOBS_Everage = 0,LeftOBS_Everage = 0, RightOBS_Everage = 0;
	static int32_t FrontOBS_Sum = 0, LeftOBS_Sum = 0, RightOBS_Sum = 0;
	int16_t OBS_Diff = 350;

	/*---------------Front-----------------------*/
	Front_OBS_Buffer = Get_FrontOBS();

	FrontOBS_Sum += Front_OBS_Buffer;
	FrontOBS_E_Counter++;
	FrontOBS_Everage = FrontOBS_Sum / FrontOBS_E_Counter;
	if (ABS_Minus(FrontOBS_Everage, Front_OBS_Buffer) > 50) {
		FrontOBS_E_Counter = 0;
		FrontOBS_Sum = 0;
	}
	if (FrontOBS_E_Counter > Cy) {
		FrontOBS_E_Counter = 0;
		Front_OBS_Buffer = Front_OBSTrig_Value - OBS_Diff;
		Front_OBS_Buffer = (FrontOBS_Everage + Front_OBS_Buffer) / 2;
		if (Front_OBS_Buffer < 100) {
			Front_OBS_Buffer = 100;
		}
		Front_OBSTrig_Value = Front_OBS_Buffer + OBS_Diff;
		//Beep(1, 10, 0, 1);
		//ROS_INFO("Update Front_OBSTrig_Value = %d.", Front_OBSTrig_Value);
	}

	/*---------------Left-----------------------*/
	Left_OBS_Buffer = Get_LeftOBS();

	LeftOBS_Sum += Left_OBS_Buffer;
	LeftOBS_E_Counter++;
	LeftOBS_Everage = LeftOBS_Sum / LeftOBS_E_Counter;
	if (ABS_Minus(LeftOBS_Everage, Left_OBS_Buffer) > 50) {
		LeftOBS_E_Counter = 0;
		LeftOBS_Sum = 0;
	}
	if (LeftOBS_E_Counter > Cy) {
		LeftOBS_E_Counter = 0;
		Left_OBS_Buffer = Left_OBSTrig_Value - OBS_Diff;
		Left_OBS_Buffer = (LeftOBS_Everage + Left_OBS_Buffer) / 2;
		if (Left_OBS_Buffer < 100) {
			Left_OBS_Buffer = 100;
		}
		Left_OBSTrig_Value = Left_OBS_Buffer + OBS_Diff;
		//Beep(4, 10, 0, 1);
		//ROS_INFO("Update Left_OBSTrig_Value = %d.", Left_OBSTrig_Value);
	}
	/*---------------Right-----------------------*/
	Right_OBS_Buffer = Get_RightOBS();

	RightOBS_Sum += Right_OBS_Buffer;
	RightOBS_E_Counter++;
	RightOBS_Everage = RightOBS_Sum / RightOBS_E_Counter;
	if (ABS_Minus(RightOBS_Everage, Right_OBS_Buffer) > 50) {
		RightOBS_E_Counter = 0;
		RightOBS_Sum = 0;
	}
	if (RightOBS_E_Counter > Cy) {
		RightOBS_E_Counter = 0;
		Right_OBS_Buffer = Right_OBSTrig_Value - OBS_Diff;
		Right_OBS_Buffer = (RightOBS_Everage + Right_OBS_Buffer) / 2;
		if (Right_OBS_Buffer < 100) {
			Right_OBS_Buffer = 100;
		}
		Right_OBSTrig_Value = Right_OBS_Buffer + OBS_Diff;
		//Beep(8, 10, 0, 1);
		//ROS_INFO("Update Right_OBSTrig_Value = %d.", Right_OBSTrig_Value);
	}
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
	if (robot::instance()->robot_get_left_wall() > (Leftwall_OBSTrig_Vale +500)){
		return 1;
	}
	return 0;
}

void Adjust_OBST_Value(void)
{
	if(robot::instance()->robot_get_obs_front() > Front_OBSTrig_Value )
		Front_OBSTrig_Value += 800;
	if(robot::instance()->robot_get_obs_left() > Left_OBSTrig_Value)
		Left_OBSTrig_Value	+= 800;
	if(robot::instance()->robot_get_obs_right() > Right_OBSTrig_Value)
		Right_OBSTrig_Value += 800;
}

void Reset_OBST_Value(void)
{
	Left_OBSTrig_Value = robot::instance()->robot_get_obs_front() + 1000;
	Front_OBSTrig_Value = robot::instance()->robot_get_obs_left() + 1000;
	Right_OBSTrig_Value = robot::instance()->robot_get_obs_right() + 1000;
}

uint8_t Spot_OBS_Status(void)
{
	uint8_t status =0;
	if(robot::instance()->robot_get_obs_left() > 1000)status |=Status_Left_OBS;
	if(robot::instance()->robot_get_obs_right() > 1000)status |=Status_Right_OBS;
	if(robot::instance()->robot_get_obs_front() >1500)status |=Status_Front_OBS;
	return status;
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

void Move_Forward(uint8_t Left_Speed, uint8_t Right_Speed)
{
	wheel_left_direction = 0;
	wheel_right_direction = 0;

	Set_Wheel_Speed(Left_Speed, Right_Speed);
}

uint8_t Get_VacMode(void)
{
	// Return the vacuum mode
	return Vac_Mode;
}

void Switch_VacMode(void)
{
	// Switch the vacuum mode between Max and Normal
	if (Get_VacMode() == Vac_Normal){
		Set_VacMode(Vac_Max);
	}else{
		Set_VacMode(Vac_Normal);
	}
	// Process the vacuum mode
	Set_Vac_Speed();
}

void Set_Gyro_Status(void)
{
	Gyro_Status = 1;
}

void Reset_Gyro_Status(void)
{
	Gyro_Status = 0;
}

uint8_t Is_Gyro_On(void)
{
	return Gyro_Status;
}

void Set_Rcon_Status(uint32_t code)
{
	Rcon_Status = code;
}

void Reset_Rcon_Status(void)
{
	Rcon_Status = 0;
}

uint32_t Get_Rcon_Status(){
	//Rcon_Status = robot::instance()->robot_get_rcon();
	return Rcon_Status;
}

/*----------------------------------------Remote--------------------------------*/
uint8_t Remote_Key(uint8_t key)
{
	// Debug
	if (Remote_Status > 0)
	{
		ROS_INFO("%s, %d Remote_Status = %x",__FUNCTION__,__LINE__, Remote_Status);
	}
	if(Remote_Status & key)
	{
		Beep(2, 2, 0, 1);
		return 1;
	}
	else
	{
		return 0;
	}
}
void Set_Rcon_Remote(uint8_t cmd)
{
	Remote_Status |= cmd;
}
void Reset_Rcon_Remote(void)
{
	Remote_Status = 0;
}

uint8_t Get_Rcon_Remote(void)
{
	return Remote_Status;
}

void Set_MoveWithRemote(void)
{
	remote_move_flag = 1;
}

void Reset_MoveWithRemote(void)
{
	remote_move_flag = 0;
}

uint8_t Check_Bat_SetMotors(uint32_t Vacuum_Voltage, uint32_t Side_Brush, uint32_t Main_Brush)
{
	static uint8_t low_acc=0;
	if(Check_Bat_Stop()){
		if(low_acc<255)
			low_acc++;
		if(low_acc>50){
			low_acc = 0;
			uint16_t t_vol = GetBatteryVoltage();
			uint8_t v_pwr = Vacuum_Voltage/t_vol;
			uint8_t s_pwr = Side_Brush/t_vol;
			uint8_t m_pwr = Main_Brush/t_vol;

			Set_BLDC_Speed(v_pwr);
			Set_SideBrush_PWM(s_pwr,s_pwr);
			Set_MainBrush_PWM(m_pwr);
			return 1;
			
		}
		else
			return 0;
	}
	else{
		return 0;
	}
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
	// Set the brightnesss of the LED within range(0, 100).
	G = G < 100 ? G : 100;
	R = R < 100 ? R : 100;
	control_set(CTL_LED_RED, R & 0xff);
	control_set(CTL_LED_GREEN, G & 0xff);
}

void Stop_Brifly(void)
{
	//ROS_INFO("%s %d: stopping robot.", __FUNCTION__, __LINE__);
	do {
		Set_Wheel_Speed(0, 0);
		usleep(15000);
		//ROS_INFO("%s %d: linear speed: (%f, %f, %f)", __FUNCTION__, __LINE__,
		//	robot::instance()->robot_get_linear_x(), robot::instance()->robot_get_linear_y(), robot::instance()->robot_get_linear_z());
	} while (robot::instance()->robot_is_moving());
	//ROS_INFO("%s %d: robot is stopped.", __FUNCTION__, __LINE__);
}

void Set_MainBrush_PWM(uint16_t PWM)
{
	// Set main brush PWM, the value of PWM should be in range (0, 100).
	PWM = PWM < 100 ? PWM : 100;
	control_set(CTL_BRUSH_MAIN, PWM & 0xff);
}

void Set_SideBrush_PWM(uint16_t L, uint16_t R)
{
	// Set left and right brush PWM, the value of L/R should be in range (0, 100).
	L = L < 100 ? L : 100 ;
	LBrush_PWM = L;
	control_set(CTL_BRUSH_LEFT, L & 0xff);
	R = R < 100 ? R : 100 ;
	RBrush_PWM = R;
	control_set(CTL_BRUSH_RIGHT, R & 0xff);
}

uint8_t Get_LeftBrush_Stall(void)
{
	return 0;
}

uint8_t Get_RightBrush_Stall(void)
{
	return 0;
}




uint8_t Get_Touch_Status(void)
{
	return Touch_Status;
}

void Reset_Touch(void)
{
	Touch_Status = 0;
}

void Set_Touch(void)
{
	Touch_Status = 1;
}

void Deceleration(void)
{
}

uint8_t Touch_Detect(void)
{
	// Get the key value from robot sensor
	if (Get_Touch_Status()){
		ROS_WARN("Touch status == 1");
		return 1;
	}
	if (Remote_Key(Remote_Clean)){
		ROS_WARN("Remote_Key clean.");
		Reset_Rcon_Remote();
		Set_Touch();
		return 1;
	}
	if (Get_Cliff_Trig() == 0x07){
		ROS_WARN("Cliff triggered.");
		Set_Touch();
		return 1;
	}

	return 0;
}

uint8_t Is_Station(void)
{
	if (Get_Rcon_Status() & 0x777777) // It means six rcon accepters receive any of the charger stub signal.
	{
		return 1;
	}
	return 0;
}

uint8_t Is_ChargerOn(void)
{
	if (robot::instance()->robot_get_charge_status() == 1){
		return 1;
	}else{
		return 0;
	}
}

uint8_t Is_Water_Tank(void)
{
	return 0;
}


void Set_Clean_Mode(uint8_t mode)
{
	Cleaning_mode = mode;
}

void Beep(uint8_t Sound_Code, int Sound_Time_Count, int Silence_Time_Count, int Total_Time_Count)
{
	// Sound_Code means the interval of the speaker sounding, higher interval makes lower sound.
	robotbase_sound_code = Sound_Code;
	// Total_Time_Count means how many loops of speaker sound loop will it sound.
	robotbase_speaker_sound_loop_count = Total_Time_Count;
	// A speaker sound loop contains one sound time and one silence time
	// Sound_Time_Count means how many loops of sendStream loop will it sound in one speaker sound loop
	robotbase_speaker_sound_time_count = Sound_Time_Count;
	// Silence_Time_Count means how many loops of sendStream loop will it be silence in one speaker sound loop, -1 means consistently beep.
	robotbase_speaker_silence_time_count = Silence_Time_Count;
	// Trigger the update flag to start the new beep action
	robotbase_beep_update_flag = true;
}

void Initialize_Motor(void)
{
#ifdef BLDC_INSTALL
	Clear_BLDC_Fail();
	BLDC_OFF;
	delay(5000);
	Set_BLDC_TPWM(40);
	Set_Vac_Speed();
#endif
	Set_MainBrush_PWM(50);
	Set_SideBrush_PWM(60,60);
	Set_BLDC_Speed(40);
//	Move_Forward(0,0);
	Stop_Brifly();
	Reset_TempPWM();
//	Left_Wheel_Slow=0;
//	Right_Wheel_Slow=0;
//	Reset_Bumper_Error();
}

void Disable_Motors(void)
{
	// Disable all the motors, including brush, wheels, and vacuum.
	// Stop the wheel
	Set_Wheel_Speed(0, 0);
	// Stop the side brush
	Set_SideBrush_PWM(0, 0);
	// Stop the main brush
	Set_MainBrush_PWM(0);
	// Stop the vacuum, directly stop the BLDC
	Set_BLDC_Speed(0);
}

void set_start_charge(void)
{
	// This function will turn on the charging function.
	control_set(CTL_CHARGER, 0x01);
}

void set_stop_charge(void)
{
	// Set the flag to false so that it can quit charger mode.
	control_set(CTL_CHARGER, 0x00);
}

void set_main_pwr(uint8_t val)
{
	control_set(CTL_MAIN_PWR, val & 0xff);
}


void Set_CleanTool_Power(uint8_t vacuum_val,uint8_t left_brush_val,uint8_t right_brush_val,uint8_t main_brush_val)
{
	int vacuum_pwr = vacuum_val;
	vacuum_pwr = vacuum_pwr > 0 ? vacuum_pwr : 0;
	vacuum_pwr = vacuum_pwr < 100 ? vacuum_pwr : 100;
	control_set(CTL_VACCUM_PWR, vacuum_pwr & 0xff);

	int brush_left = left_brush_val;
	control_set(CTL_BRUSH_LEFT, brush_left & 0xff);

	int brush_right = right_brush_val;
	control_set(CTL_BRUSH_RIGHT, brush_right & 0xff);

	int brush_main = main_brush_val;
	control_set(CTL_BRUSH_MAIN, brush_main & 0xff);
}

void control_set(uint8_t type, uint8_t val)
{
	if(!IsSendBusy()){
		if (type >= CTL_WHEEL_LEFT_HIGH && type <= CTL_GYRO) {
			sendStream[type] = val;
			//sendStream[SEND_LEN-3] = calcBufCrc8((char *)sendStream, SEND_LEN-3);
			//serial_write(SEND_LEN, sendStream);
		}
	}
}

void control_append_crc(){
	if(!IsSendBusy()){
		sendStream[CTL_CRC] = calcBufCrc8((char *)sendStream, SEND_LEN-3);
	}	
}

void control_stop_all(void)
{
	uint8_t i;
	if(!IsSendBusy()){
		for(i = 2; i < (SEND_LEN)-2; i++) {
			if (i == CTL_MAIN_PWR)
				sendStream[i] = 0x01;
			else
				sendStream[i] = 0x00;
		}
	}
	//sendStream[SEND_LEN-3] = calcBufCrc8((char *)sendStream, SEND_LEN-3);
	//serial_write(SEND_LEN, sendStream);
}

uint8_t IsSendBusy(void)
{
	return sendflag;
}

void SetSendFlag(void)
{
	sendflag = 1;
}

void ResetSendFlag(void)
{
	sendflag = 0;
}
void Random_Back(void)
{
	Stop_Brifly();
	Quick_Back(12,30);
	
}

void Move_Back(void)
{
	Stop_Brifly();
	Quick_Back(18,30);
}

void Back(void)
{
	Stop_Brifly();
	Quick_Back(18,30);
}

void Cliff_Move_Back()
{
	Stop_Brifly();
	Quick_Back(18,60);
}
void Set_RightWheel_Step(uint32_t step)
{
	right_wheel_step = step;
}

void Set_LeftWheel_Step(uint32_t step)
{
	left_wheel_step = step;
}
void Reset_RightWheel_Step()
{
	rw_t = ros::Time::now();
	right_wheel_step = 0;
}

void Reset_LeftWheel_Step()
{
	lw_t = ros::Time::now();
	left_wheel_step	= 0;
}

uint16_t GetBatteryVoltage()
{
	return robot::instance()->robot_get_battery_voltage();
}

uint8_t  Check_Bat_Stop()
{
	if(GetBatteryVoltage()<LOW_BATTERY_STOP_VOLTAGE)
		return 1;
	else
		return 0;
}

void Set_Key_Press(uint8_t key)
{
	Key_Status |= key;
}

void Reset_Key_Press(uint8_t key)
{
	Key_Status &= ~key;
}

uint8_t Get_Key_Press(void)
{	
	return Key_Status;
}

uint8_t Get_Key_Time(uint16_t key)
{
	// This time is count for 100ms.
	uint8_t time = 0;
	while(ros::ok()){
		time++;
		if(time>20)break;
		usleep(100000);
		if(Get_Key_Press()!=key)break;
	}
	return time;
}

uint8_t Is_Front_Close(){	
	if(robot::instance()->robot_get_obs_front() > Front_OBSTrig_Value+1500)
		return 1;
	else
		return 0;
}

uint8_t Is_virtualWall(void){
	return 0;
}

uint8_t Is_Turn_Remote(void)
{
	if (Remote_Key(Remote_Max | Remote_Home | Remote_Spot | Remote_Wall_Follow))
	{
		Reset_Rcon_Remote();
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t Get_Direction_Flag(void)
{
	return Direction_Flag;
}

void Set_Direction_Flag(uint8_t flag)
{
	Direction_Flag = flag;
}

uint8_t Is_Direction_Right(void)
{
	if(Get_Direction_Flag() == Direction_Flag_Right)return 1;
	return 0;
}
uint8_t Is_Direction_Left(void)
{
	if(Get_Direction_Flag() == Direction_Flag_Left)return 1;
	return 0;
}
uint8_t Is_LeftWheel_Reach(int32_t step)
{
	if(left_wheel_step > (uint32_t)step)
		return 1;
	else
		return 0;
}

uint8_t Is_RightWheel_Reach(int32_t step)
{
	if(right_wheel_step > (uint32_t)step)
		return 1;
	else
		return 0;
}

void Wall_Move_Back(void)
{
	uint16_t count=0;
	uint16_t tp = 0;
	uint8_t mc = 0;	
	Set_Dir_Backward();
	Set_Wheel_Speed(3,3);
	usleep(20000);
	Reset_Wheel_Step();
	while(((left_wheel_step<100)||(right_wheel_step<100))&&ros::ok()){
		tp = left_wheel_step/3+8;
		if(tp>12)tp=12;	
		Set_Wheel_Speed(tp,tp);
		usleep(1000);
	
		if(Touch_Detect())
			return;
		count++;
		if(count>3000);
			return;	
		mc = Check_Motor_Current();
		if(mc == Check_Left_Wheel || mc == Check_Right_Wheel)
			return;
	}	
	Set_Dir_Forward();
	Set_Wheel_Speed(0,0);
		
}

void Reset_Move_Distance(){
	MoveStepCounter = 0;
}

uint8_t Is_Move_Finished(int32_t distance)
{
	MoveStepCounter = Get_LeftWheel_Step();
	MoveStepCounter +=Get_RightWheel_Step();
	if((MoveStepCounter/2)>distance)
		return 1;
	else
		return 0;
}
uint32_t Get_Move_Distance(void)
{
	MoveStepCounter = Get_LeftWheel_Step();
	MoveStepCounter +=Get_RightWheel_Step();

	if(MoveStepCounter>0)return (uint32_t)(MoveStepCounter/2);
	return 0;
}

void OBS_Turn_Left(uint16_t speed,uint16_t angle)
{
	uint16_t counter = 0;
	uint8_t oc = 0;
	Set_Dir_Left();
	Reset_Rcon_Remote();
	Set_Wheel_Speed(speed,speed);
	Reset_LeftWheel_Step();
	while(ros::ok()&&left_wheel_step <angle){
		counter++;
		if(counter>3000)
			return;
		if(Is_Turn_Remote())
			return;
		oc = Check_Motor_Current();
		if(oc== Check_Left_Wheel || oc==Check_Right_Wheel)
			return;
		usleep(1000);
	}
		
}
void OBS_Turn_Right(uint16_t speed,uint16_t angle)
{
	uint16_t counter = 0;
	uint8_t oc = 0;
	Set_Dir_Right();
	Reset_Rcon_Remote();
	Set_Wheel_Speed(speed,speed);
	Reset_RightWheel_Step();
	while(ros::ok()&&right_wheel_step <angle){
		counter++;
		if(counter>3000)
			//return;
			break;
		if(Is_Turn_Remote())
			//return;
			break;
		oc = Check_Motor_Current();
		if(oc== Check_Left_Wheel || oc==Check_Right_Wheel)
			//return;
			break;
		usleep(1000);
	}

}

void Cliff_Turn_Left(uint16_t speed,uint16_t angle)
{
	uint16_t Counter_Watcher=0;
	//Left_Wheel_Step=0;
	Reset_TempPWM();
	Set_Wheel_Speed(speed,speed);
	Counter_Watcher=0;
	Reset_Rcon_Remote();
	int16_t target_angle;
	uint16_t gyro_angle;
	// This decides whether robot should stop when left cliff triggered.
	bool right_cliff_triggered = false;

	if (Get_Cliff_Trig() & Status_Cliff_Right)
	{
		right_cliff_triggered = true;
	}

	gyro_angle = Gyro_GetAngle();

	target_angle = gyro_angle + angle;
	if (target_angle >= 3600) {
		target_angle = target_angle - 3600;
	}

	Set_Dir_Left();
	Set_Wheel_Speed(speed, speed);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);
	while(ros::ok())
	{
		if (abs(target_angle - Gyro_GetAngle()) < 20) {
			break;
		}
		if (abs(target_angle - Gyro_GetAngle()) < 50) {
			Set_Wheel_Speed(speed / 2, speed / 2);
		} else {
			Set_Wheel_Speed(speed, speed);
		}
		//delay(10);
		usleep(1000);
		Counter_Watcher++;
		if(Counter_Watcher>3000)
		{
			if(Is_Encoder_Fail())
			{
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			return;
		}
		if(Is_Turn_Remote())return;
		if(!right_cliff_triggered && (Get_Cliff_Trig() & Status_Cliff_Right))
		{
			Stop_Brifly();
			return;
		}
		if(Touch_Detect())
		{
			return;
		}
		if((Check_Motor_Current()==Check_Left_Wheel)||(Check_Motor_Current()==Check_Right_Wheel))return;
	}
}

void Cliff_Turn_Right(uint16_t speed,uint16_t angle)
{
	uint16_t Counter_Watcher=0;
	//Left_Wheel_Step=0;
	Reset_TempPWM();
	Set_Wheel_Speed(speed,speed);
	Counter_Watcher=0;
	Reset_Rcon_Remote();
	int16_t target_angle;
	uint16_t gyro_angle;
	// This decides whether robot should stop when left cliff triggered.
	bool left_cliff_triggered = false;

	if (Get_Cliff_Trig() & Status_Cliff_Left)
	{
		left_cliff_triggered = true;
	}

	gyro_angle = Gyro_GetAngle();

	target_angle = gyro_angle - angle;
	if (target_angle < 0) {
		target_angle = 3600 + target_angle;
	}

	Set_Dir_Right();
	Set_Wheel_Speed(speed, speed);

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);
	while(ros::ok())
	{
		if (abs(target_angle - Gyro_GetAngle()) < 20) {
			break;
		}
		if (abs(target_angle - Gyro_GetAngle()) < 50) {
			Set_Wheel_Speed(speed / 2, speed / 2);
		} else {
			Set_Wheel_Speed(speed, speed);
		}
		//delay(10);
		usleep(1000);
		Counter_Watcher++;
		if(Counter_Watcher>3000)
		{
			if(Is_Encoder_Fail())
			{
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			return;
		}
		if(Is_Turn_Remote())return;
		if(!left_cliff_triggered && (Get_Cliff_Trig() & Status_Cliff_Left))
		{
			Stop_Brifly();
			return;
		}
		if(Touch_Detect())
		{
			return;
		}
		if((Check_Motor_Current()==Check_Left_Wheel)||(Check_Motor_Current()==Check_Right_Wheel))return;
	}
}

uint8_t Get_Random_Factor(void){
	srand(time(0));
	return (uint8_t)rand()%100;
}

uint8_t Is_NearStation(void){
	static uint32_t s_count=0;
	static uint32_t no_s = 0;
	static uint32_t s_rcon =0;
	s_rcon = Get_Rcon_Status();
	if(s_rcon&0x00000f00){
		return 1;
	}
	if(s_rcon&0x0330ff){
		no_s = 0;
		s_count ++;
		Reset_Rcon_Status();	
		if(s_count>3)
		{
			s_count=0;
			return 1;
		}
	}
	else{
		no_s ++;
		if(no_s > 50){
			no_s = 0;
			s_count =0;
		}
	}
	return 0;
}

void Set_Mobility_Step(uint32_t Steps)
{
	Mobility_Step = Steps;
}

void Reset_Mobility_Step()
{
	Mobility_Step = 0;
}

uint32_t  Get_Mobility_Step()
{
	return Mobility_Step;
}

void Check_Mobility(void)
{
	
}

void Add_Average(uint32_t data)
{
	Average_Move += data;
	Average_Counter++;
	if(data>Max_Move)
		Max_Move = data;
	
}

uint32_t Get_Average_Move(void)
{
	return (Average_Move/Average_Counter);
}

uint32_t Reset_Average_Counter(void)
{
	Average_Move = 0;
	Average_Counter = 0;
}

void Reset_VirtualWall(void)
{

}

uint8_t  VirtualWall_TurnRight()
{
	return 0;
}

uint8_t VirtualWall_TurnLeft()
{
	return 0;
}

uint8_t Is_WorkFinish(uint8_t m)
{
	static uint8_t bat_count = 0;
	uint32_t wt = Get_Work_Time();	
	if(m){
		if(wt>Auto_Work_Time)return 1;
	}
	else if(wt>Const_Work_Time)return 1;
	if(GetBatteryVoltage()<1420)
	{
		bat_count++;
		if(bat_count>50)return 1;
	}
	else
		bat_count = 0;
	return 0;		
}

uint8_t Get_Room_Mode(){
	return Room_Mode;	
}

void Set_Room_Mode(uint8_t m){
	if(m)
		Room_Mode = 1;
	else
		Room_Mode = 0;
}

void Reset_WallAccelerate()
{
		Wall_Accelerate =0;
}

uint32_t Get_WallAccelerate()
{
	
	return Wall_Accelerate=Get_RightWheel_Step(); 
}

/*---------------------------------Bumper Error ----------------------------------------*/
uint8_t Is_Bumper_Jamed()
{
	if(Get_Bumper_Status())
	{
		ROS_INFO("JAM1");
		Move_Back();
		if(Get_Bumper_Status())
		{
			ROS_INFO("JAM2");
			// Quick back will not set speed to 100, it will be limited by the RUN_TOP_SPEED.
			Quick_Back(100,200);
			if(Get_Bumper_Status())
			{
				ROS_INFO("JAM3");
				Jam_Turn_Right(60, 900);
				Jam_Turn_Left(60, 1800);
				if(Get_Bumper_Status())
				{
					ROS_INFO("JAM4");
					//if(Is_Bumper_Fail())
					//{
						ROS_INFO("JAM5");
						Set_Clean_Mode(Clean_Mode_Userinterface);
						Set_Error_Code(Error_Code_Bumper);
						wav_play(WAV_ERROR_BUMPER);
						return 1;
					//}
				}
			}
		}
	}
	return 0;
}

void Reset_Bumper_Error(void)
{
	Bumper_Error=0;
}
uint8_t Is_Bumper_Fail(void)
{
	Bumper_Error++;
	ROS_INFO("Buper_Error = %d", Bumper_Error);
	if(Bumper_Error>3)return 1;
	else return 0;
}

uint8_t Is_VirtualWall()
{
	return 0;
}

void Set_Gyro_On(void)
{
	if (Is_Gyro_On()){
		ROS_INFO("gyro on already");
	}
	else
	{
		control_set(CTL_GYRO, 0x02);
	}
}

bool Wait_For_Gyro_On(void)
{
	static int count=0;
	count = 0;
	ROS_INFO("waiting for gyro start");
	auto stop_angle_v = robot::instance()->robot_get_angle_v();
	while (count<10 && !except_event())
	{
		usleep(10000);

		if (robot::instance()->robot_get_angle_v() != stop_angle_v){
			++count;
		}
//		ROS_INFO("gyro start ready(%d),angle_v(%f)", count, robot::instance()->robot_get_angle_v());
	}
	if(count == 10)
	{
		ROS_INFO("gyro start ok");
		Set_Gyro_Status();
		return true;
	}
	ROS_INFO("gyro start fail");
	Reset_Gyro_Status();
	Set_Gyro_Off();
	return false;
}

void Set_Gyro_Off()
{
	if (!Is_Gyro_On()){
		ROS_INFO("gyro stop already");
		return;
	}
	static int count=0;
	control_set(CTL_GYRO, 0x00);
	ROS_INFO("waiting for gyro stop");
	count = 0;
	auto angle_v = robot::instance()->robot_get_angle_v();

	while(count <= 20)
	{
		usleep(10000);
		count++;
		if (robot::instance()->robot_get_angle_v() != angle_v){
			count=0;
			angle_v = robot::instance()->robot_get_angle_v();
		}
//		ROS_INFO("gyro stop ready(%d),angle_v(%f)", count, robot::instance()->robot_get_angle_v());
	}
	Reset_Gyro_Status();
	ROS_INFO("gyro stop ok");
}

#if GYRO_DYNAMIC_ADJUSTMENT
void Set_Gyro_Dynamic_On(void)
{
	if (Is_Gyro_On())
	{
		control_set(CTL_GYRO, 0x03);
	}
	else
	{
		control_set(CTL_GYRO, 0x01);
	}
}

void Set_Gyro_Dynamic_Off(void)
{
	if (Is_Gyro_On())
	{
		control_set(CTL_GYRO, 0x02);
	}
	else
	{
		control_set(CTL_GYRO, 0x00);
	}
}
#endif

int32_t ABS_Minus(int32_t A,int32_t B)
{
	if(A>B)
	{
		return A-B;
	}
	return B-A;
}

void ladar_gpio(char val)
{
	int fd = open("/proc/driver/wifi-pm/power", O_WRONLY);
	char buf[] = {val};
	write(fd,buf,1);
	close(fd);
}
void Check_SideBrush_Stall(void)
{
	static uint32_t Time_LBrush_Stop = 0, Time_RBrush_Stop = 0;
	static uint8_t LBrush_Stall_Counter = 0, RBrush_Stall_Counter = 0, Flag_LBrush_Is_Stall = 0, Flag_RBrush_Is_Stall = 0;

	/*---------------------------------Left Brush Stall---------------------------------*/
	if(Flag_LBrush_Is_Stall == 0 || (time(NULL) - Time_LBrush_Stop) > 5)
	{
		if(robot::instance()->robot_get_lbrush_oc())
		{
			if(LBrush_Stall_Counter < 200)
				LBrush_Stall_Counter++;
		}
		else
		{
			LBrush_Stall_Counter = 0;
			Flag_LBrush_Is_Stall = 0;
		}

		if(LBrush_Stall_Counter >= 10)
		{
			control_set(CTL_BRUSH_LEFT, 0);
			Flag_LBrush_Is_Stall = 1;
			Time_LBrush_Stop = time(NULL);
			wav_play(WAV_ERROR_LEFT_BRUSH);
		}
		else
		{
			Flag_LBrush_Is_Stall = 0;
			control_set(CTL_BRUSH_LEFT, LBrush_PWM);
		}
	}
	else
	{
		control_set(CTL_BRUSH_LEFT, 0);
	}

	/*-------------------------------Rigth Brush Stall---------------------------------*/
	if(Flag_RBrush_Is_Stall == 0 || (time(NULL) - Time_RBrush_Stop) > 5)
	{
		if(robot::instance()->robot_get_rbrush_oc())
		{
			if(RBrush_Stall_Counter < 200)
				RBrush_Stall_Counter++;
		}
		else
		{
			RBrush_Stall_Counter = 0;
			Flag_RBrush_Is_Stall = 0;
		}

		if(RBrush_Stall_Counter >= 10)
		{
			control_set(CTL_BRUSH_RIGHT, 0);
			Flag_RBrush_Is_Stall = 1;
			Time_RBrush_Stop = time(NULL);
			wav_play(WAV_ERROR_RIGHT_BRUSH);
		}
		else
		{
			Flag_RBrush_Is_Stall = 0;
			control_set(CTL_BRUSH_RIGHT, RBrush_PWM);
		}
	}
	else
	{
		control_set(CTL_BRUSH_RIGHT, 0);
	}
}

void Set_Plan_Status(bool Status)
{
	Plan_Status = Status;
}

bool Get_Plan_Status()
{
	return Plan_Status;
}

