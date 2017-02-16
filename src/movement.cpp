#include <stdint.h>
#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include "robot.hpp"
#include <time.h>

#include "gyro.h"
#include "movement.h"
#include "crc8.h"
#include "serial.h"
#include "robotbase.h"
#include "config.h"
#define MOVEMENT "movement"
extern uint8_t sendStream[SEND_LEN];

static int16_t Left_OBSTrig_Value = 500;
static int16_t Front_OBSTrig_Value = 500;
static int16_t Right_OBSTrig_Value = 500;
static int16_t Leftwall_OBSTrig_Vale = 500;
static uint8_t wheel_left_direction = 0;
static uint8_t wheel_right_direction = 0;
static uint8_t remote_move_flag=0;
static uint8_t home_remote_flag = 0;
static uint32_t Rcon_Status;

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

static int32_t MoveStepCounter = 0;
static uint32_t Mobility_Step = 0;
static uint8_t Direction_Flag=0;
// Variable for vacuum mode

volatile uint8_t Vac_Mode;
static uint8_t Cleaning_mode = 0;
static uint8_t sendflag=0;
static time_t work_time;
ros::Time lw_t,rw_t; // these variable is used for calculate wheel step
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
	code = code;
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
	step = rwsp*t/0.181;
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
	step = lwsp*t/0.181;
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
	return leftwall_step;
}
uint32_t Get_RightWall_Step(void)
{
	return rightwall_step;
}
void Set_Wheel_Step(uint32_t Left, uint32_t Right)
{
	left_wheel_step = Left;
	right_wheel_step = Right;
}

int32_t Get_Wall_ADC(void)
{
	return (int32_t)robot::instance()->robot_get_wall();
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
	// Quickly move back for a distance.
	wheel_left_direction = 1;
	wheel_right_direction = 1;
	Reset_Wheel_Step();
	Set_Wheel_Speed(Speed, Speed);
	// This count is for how many milliseconds it should take. The Distance is in mm.
	int back_count = int(1000 * Distance / (Speed * 7.23));
	//printf("[movement.cpp] Quick_back for %dms.\n", back_count);
	for (int i = 0; i < back_count; i++){
		// Sleep for 1 millisecond
		usleep(1000);
	}
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
	
	uint8_t oc=0;
	printf("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(0), speed);
	while (ros::ok()) {
		if (abs(target_angle - Gyro_GetAngle(0)) < 20) {
			break;
		}
		if (abs(target_angle - Gyro_GetAngle(0)) < 50) {
			Set_Wheel_Speed(speed / 2, speed / 2);
		} else {
			Set_Wheel_Speed(speed, speed);
		}
		oc= Check_Motor_Current();
		if(oc == Check_Left_Wheel || oc== Check_Right_Wheel)
			break;
		if(Touch_Detect())
			break;
		if(Is_Turn_Remote())
			break;
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
	uint8_t oc=0;
	printf("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(0), speed);
	while (ros::ok()) {
		if (abs(target_angle - Gyro_GetAngle(0)) < 20) {
			break;
		}
		if (abs(target_angle - Gyro_GetAngle(0)) < 50) {
			Set_Wheel_Speed(speed / 2, speed / 2);
		} else {
			Set_Wheel_Speed(speed, speed);
		}
		oc= Check_Motor_Current();
		if(oc == Check_Left_Wheel || oc== Check_Right_Wheel)
			break;
		if(Touch_Detect())
			break;
		if(Is_Turn_Remote())
			break;
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
uint8_t Spot_OBS_Status(void)
{
	uint8_t status =0;
	if(robot::instance()->robot_get_obs_left() > 1500)status |=Status_Left_OBS;
	if(robot::instance()->robot_get_obs_right() > 1500)status |=Status_Right_OBS;
	if(robot::instance()->robot_get_obs_front() >2000)status |=Status_Front_OBS;
	return status;
}

int32_t Get_FrontOBS(void)
{
	return (int32_t)robot::instance()->robot_get_obs_front();
}

int32_t Get_LeftOBS(void)
{
	return (int32_t)robot::instance()->robot_get_obs_left();
}
int32_t Get_rightOBS(void)
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
		ROS_DEBUG_NAMED(MOVEMENT,"Left cliff is detected:%d", cl);
		Cliff_Status += 0x01;
	}
	if (cr< Cliff_Limit){
		ROS_DEBUG_NAMED(MOVEMENT,"Right cliff is detected:%d", cr);
		Cliff_Status += 0x02;
	}
	if (cf < Cliff_Limit){
		ROS_DEBUG_NAMED(MOVEMENT,"Front cliff is detected:%d", cf);
		Cliff_Status += 0x04;
	}
	if (Cliff_Status != 0x00){
		ROS_DEBUG_NAMED(MOVEMENT,"Return Cliff status:%x.", Cliff_Status);
	}
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
						Cliff_Turn_Right(100,300);
						break;
				case 2:
						Cliff_Turn_Left(100,300);
						break;
				case 3:
						Cliff_Turn_Right(100,300);
						break;
				case 4:
						Cliff_Turn_Left(100,300);
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
}

uint8_t Cliff_Event(uint8_t event)
{
	uint16_t temp_adjust=0,random_factor=0;
	uint8_t d_flag = 0;
	if(left_wheel_step%2)temp_adjust = 450;
	else temp_adjust = 0;
	if(right_wheel_step%3)random_factor = 1;
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
	printf("[movement.cpp] Get charge status: %d.\n", robot::instance()->robot_get_charge_status());
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
		return 1;
	}
	// Start turning left.
	target_angle = Gyro_GetAngle(0) - 120;
	if (target_angle < 0) {
		target_angle = 3600 + target_angle;
	}
	wheel_left_direction = 0;
	wheel_right_direction = 1;
	Set_Wheel_Speed(speed, speed);
	while(abs(target_angle - Gyro_GetAngle(0)) > 20)
	{
		if(Is_ChargerOn())
		{
			Disable_Motors();
			Stop_Brifly();
			if(Is_ChargerOn())
			{
				ROS_INFO("[movement.cpp] Turn left reach charger.");
				return 1;
			}
			break;
		}
		if(Touch_Detect())
		{
			Disable_Motors();
			return 0;
		}
	}
	Stop_Brifly();
	// Start turning right.
	target_angle = Gyro_GetAngle(0) + 240;
	if (target_angle < 0) {
		target_angle = 3600 + target_angle;
	}
	wheel_left_direction = 1;
	wheel_right_direction = 0;
	Set_Wheel_Speed(speed, speed);
	while(abs(target_angle - Gyro_GetAngle(0)) > 20)
	{
		if(Is_ChargerOn())
		{
			Disable_Motors();
			Stop_Brifly();
			if(Is_ChargerOn())
			{
				ROS_INFO("[movement.cpp] Turn right reach charger.");
				return 1;
			}
			break;
		}
		if(Touch_Detect())
		{
			Disable_Motors();
			return 0;
		}
	}
	Disable_Motors();
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
	Left = Left < RUN_TOP_SPEED ? Left : RUN_TOP_SPEED;
	Right = Right < RUN_TOP_SPEED ? Right : RUN_TOP_SPEED;
	Set_LeftWheel_Speed(Left);
	Set_RightWheel_Speed(Right);
}

void Set_LeftWheel_Speed(uint8_t speed)
{
	int16_t l_speed;
	speed = speed>RUN_TOP_SPEED?RUN_TOP_SPEED:speed;
	l_speed = (int16_t)(speed*7.23);
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
	r_speed = (int16_t)(speed*7.23);
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
	Set_SideBrush_PWM(30, 30);
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
			return Check_Left_Wheel;
		}
	}
	else
		lwheel_oc_count = 0;
	if((uint32_t)robot::instance()->robot_get_rwheel_current() > Wheel_Stall_Limit){
		rwheel_oc_count++;
		if(rwheel_oc_count > 40){
			rwheel_oc_count = 0;
			return Check_Right_Wheel;
		}
	}
	else
		rwheel_oc_count = 0;
	if(robot::instance()->robot_get_rbrush_oc())
		return Check_Right_Brush;
	if(robot::instance()->robot_get_lbrush_oc())
		return Check_Left_Brush;
	if(robot::instance()->robot_get_mbrush_oc())
		return Check_Main_Brush;
	if(robot::instance()->robot_get_vacuum_oc())
		return Check_Vacuum;	
	return 0;
}

uint8_t Self_Check(uint8_t Check_Code)
{
	Check_Code = Check_Code;
	return 0;
}

uint8_t Check_Bat_Home(void)
{
	// Check if battary is lower than the low battery go home voltage value.
	if (robot::instance()->robot_get_battery_voltage() > 0 && robot::instance()->robot_get_battery_voltage() < LOW_BATTERY_GO_HOME_VOLTAGE){
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
				//printf("[movement.cpp] Work time more than 2 hours.\n");
				Set_BLDC_Speed(Vac_Speed_NormalL);
			}
		}
	}
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
	if (robot::instance()->robot_get_wall() > (Leftwall_OBSTrig_Vale +500)){
		return 1;
	}
    return 0;
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

void Set_Rcon_Status(uint32_t code)
{
	Rcon_Status = code;
}

void Reset_Rcon_Status(void)
{
	Rcon_Status = 0;
}

uint32_t Get_Rcon_Status(){
	Rcon_Status = robot::instance()->robot_get_rcon();
	return Rcon_Status;
}

uint32_t Get_Rcon_Remote(void)
{
	uint8_t ir_cmd;
	ir_cmd = robot::instance()->robot_get_ir_ctrl();
	if(ir_cmd == 0x80)
		return Remote_Forward;
	else if(ir_cmd == 0x40)
		return Remote_Left;
	else if(ir_cmd == 0x20)
		return Remote_Right;
	else if(ir_cmd == 0x10)
		return Remote_Max;
	else if(ir_cmd == 0x08)
		return Remote_Clean;
	else if(ir_cmd == 0x04)
		return Remote_Home;
	else if(ir_cmd == 0x02)
		return Remote_Random;
	else if(ir_cmd == 0x01)
		return Remote_Spot;
	else 
		return 0;
}
void Set_Rcon_Remote(uint8_t cmd)
{
	robot::instance()->robot_set_ir_cmd(cmd);
}
void Reset_Rcon_Remote(void)
{
	Set_Rcon_Remote(0);
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
	if(Check_Battery()==0){
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


void Display_Battery_Status(uint8_t display_mode)
{
	// Change the LED according to the display_mode.
	switch (display_mode){
		case Display_Clean:{
			break;
		}
		case Display_Wall:{
			break;
		}
		case Display_Zizag:{
			break;
		}
		case Display_Remote:{
			break;
		}
		case Display_Full:{
			Set_LED(100, 0);
			break;
		}
		case Display_Low:{
			Set_LED(50, 100);
			break;
		}
		default:{
			break;
		}
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
	//printf("%s %d: stopping robot.\n", __FUNCTION__, __LINE__);
	do {
		Set_Wheel_Speed(0, 0);
		usleep(15000);
		//printf("%s %d: linear speed: (%f, %f, %f)\n", __FUNCTION__, __LINE__,
		//	robot::instance()->robot_get_linear_x(), robot::instance()->robot_get_linear_y(), robot::instance()->robot_get_linear_z());
	} while (robot::instance()->robot_is_moving());
	//printf("%s %d: robot is stopped.\n", __FUNCTION__, __LINE__);
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
	control_set(CTL_BRUSH_LEFT, L & 0xff);
	R = R < 100 ? R : 100 ;
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


uint8_t Remote_Key(uint32_t key)
{
	
	if(Get_Rcon_Remote() == key)
		return 1;
	else
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
	// Get the key value from robot sensor
	if (robot::instance()->robot_get_key() == 1){
		return 1;
	}
	if (Remote_Key(Remote_Clean)){
		return 1;
	}
	if (Get_Cliff_Trig() == 0x07){
		return 1;
	}

	return 0;
}

uint8_t Is_Station(void)
{
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

void Initialize_Motors(void)
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

void set_gyro(uint8_t state, uint8_t calibration)
{
	//control_set(CTL_GYRO, (state ? 0x02 : 0x0) | (calibration ? 0x01 : 0x00));
	uint8_t du = calibration;
	control_set(CTL_GYRO, (state ? 0x02 : 0x0));

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
	Quick_Back(8,30);
	
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

uint8_t  Check_Battery()
{
	if(robot::instance()->robot_get_battery_voltage()<Low_Battery_Limit)
		return 0;
	else
		return 1;
}

uint8_t Get_Key_Press(void)
{	
	uint8_t status=0;
	if(robot::instance()->robot_get_key())
		return status |= KEY_CLEAN;
	return status;
}

uint8_t Get_Key_Time(uint16_t key)
{
	uint8_t time = 0;
	while(ros::ok()){
		time++;
		if(time>200)break;
		usleep(10000);
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

uint8_t Is_BumperJamed(void){
	return 0;
}

uint8_t Is_Turn_Remote(void){
	uint32_t rectrl = Get_Rcon_Remote();
	if(rectrl == Remote_Max)return 1;
	else if(rectrl == Remote_Home)return 1;
	else if(rectrl == Remote_Spot)return 1;
	else return 0;	
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
			return;
		if(Is_Turn_Remote())
			return;
		oc = Check_Motor_Current();
		if(oc== Check_Left_Wheel || oc==Check_Right_Wheel)
			return;
		usleep(1000);
	}

}

void Cliff_Turn_Left(uint16_t speed,uint16_t angle)
{
	OBS_Turn_Left(speed,angle);
}

void Cliff_Turn_Right(uint16_t speed,uint16_t angle)
{
	OBS_Turn_Right(speed,angle);
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

void Adjust_OBST_Value(void)
{
	
	if(robot::instance()->robot_get_obs_front() > Front_OBSTrig_Value )
		Front_OBSTrig_Value += 800;
	if(robot::instance()->robot_get_obs_left() > Left_OBSTrig_Value)
		Left_OBSTrig_Value  += 800;
	if(robot::instance()->robot_get_obs_right() > Right_OBSTrig_Value)
		Right_OBSTrig_Value += 800;
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
	if(robot::instance()->robot_get_battery_voltage()<1420)
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

uint8_t Is_Bumper_Jamed()
{
	return 0;
}

uint8_t Is_VirtualWall()
{
	return 0;
}
/*
*	@brief
*		go straight forward or backward 
*	@input
*		speed - wheel speed in mm/s
*	@output
*	@retval
*
*/
void movement_go(int16_t speed)
{
	uint8_t high_byte = speed >> 8&0xff;
	uint8_t low_byte  = speed & 0xff;

	control_set(CTL_WHEEL_LEFT_HIGH, high_byte);
	control_set(CTL_WHEEL_LEFT_LOW, low_byte);
	control_set(CTL_WHEEL_RIGHT_HIGH, high_byte);
	control_set(CTL_WHEEL_RIGHT_LOW, low_byte);
}

/*
*	@brief
*		turn left or right
*	@input
*		left_speed - left wheel speed in mm/s
*		right_speed - right wheel speed in mm/s
*	@output
*	@retval
*		int8_t - -1 error,0 normal
*
*/
void movement_turn(int16_t left,int16_t right)
{
	int16_t left_speed,right_speed;
	left_speed = (left>=0)?left:((~left)|0x8000+1);
	right_speed = (right>=0)?right:((~right)|0x8000+1);
	control_set(CTL_WHEEL_LEFT_HIGH, (left_speed>>8 & 0xff));
	control_set(CTL_WHEEL_LEFT_LOW, (left_speed & 0xff));
	control_set(CTL_WHEEL_RIGHT_HIGH, (right_speed >>8 & 0xff));
	control_set(CTL_WHEEL_RIGHT_LOW, (right_speed & 0xff));

}

/*
*	@brief
*		rotation left,around robot center
*	@input
*		speed - wheel speed in mm/s
*		right_speed - right wheel speed in mm/s
*		angle - rotating angle in 0.1 degrees
*	@output
*	@retval
*
*/
void movement_rot_left(int16_t speed)
{
	if(speed<0)
		speed = abs(speed);
	movement_turn(-speed,speed);

}

/*
*	@brief
*		rotation right,around robot center
*	@input
*		speed - wheel speed in mm/s
*	@output
*	@retval
*
*/
void movement_rot_right(int16_t speed)
{
	if(speed<0)
		speed = abs(speed);
	movement_turn(speed,-speed);

}

/*
*	@brief
*		stop move
*	@input
*	@output
*	@retval
*
*/
void movement_stop()
{
	movement_turn(0,0);
}
