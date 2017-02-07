#include <stdint.h>
#include <math.h>
#include "robot.hpp"

#include "gyro.h"
#include "movement.h"
#include "crc8.h"
#include "serial.h"
#include "robotbase.h"

extern uint8_t sendStream[SEND_LEN];

static int16_t Left_OBSTrig_Value = 500;
static int16_t Front_OBSTrig_Value = 500;
static int16_t Right_OBSTrig_Value = 500;
static int16_t Leftwall_OBSTrig_Vale = 500;
static uint8_t wheel_left_direction = 0;
static uint8_t wheel_right_direction = 0;
static uint8_t ir_cmd;
static uint8_t remote_move_flag=0;
static uint8_t home_remote_flag = 0;
static uint32_t Rcon_Status;
int8_t Left_Wheel_Speed = 0;
int8_t Right_Wheel_Speed = 0;
// Variable for vaccum mode
volatile uint8_t Vac_Mode;
static uint8_t Cleaning_mode = 0;
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

void Reset_Wheel_Step(void)
{}

void Reset_Wall_Step(void)
{}

uint32_t Get_LeftWall_Step(void)
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
	// Quickly move back for a distance.
	wheel_left_direction = 1;
	wheel_right_direction = 1;
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
	// Logic of getting the cliff status.
	uint8_t Cliff_Status = 0x00;
	if (robot::instance()->robot_get_cliff_left() < Cliff_Limit){
		printf("[movement.cpp] Left cliff is detected.\n");
		Cliff_Status += 0x01;
	}
	if (robot::instance()->robot_get_cliff_right() < Cliff_Limit){
		printf("[movement.cpp] Right cliff is detected.\n");
		Cliff_Status += 0x02;
	}
	if (robot::instance()->robot_get_cliff_right() < Cliff_Limit){
		printf("[movement.cpp] Front cliff is detected.\n");
		Cliff_Status += 0x04;
	}
	if (Cliff_Status != 0x00){
		printf("[movement.cpp] Return Cliff status:%x.\n", Cliff_Status);
	}
	return Cliff_Status;
}

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
	control_set(CTL_WHEEL_LEFT_HIGH, (left_speed >> 8) & 0xff);
	control_set(CTL_WHEEL_LEFT_LOW, left_speed & 0xff);
	control_set(CTL_WHEEL_RIGHT_HIGH, (right_speed >> 8) & 0xff);
	control_set(CTL_WHEEL_RIGHT_LOW, right_speed & 0xff);
}

void Set_LeftWheel_Speed(uint8_t speed)
{
	if(speed>100)speed=100;
	Left_Wheel_Speed = speed;
}

void Set_RightWheel_Speed(uint8_t speed)
{
	if(speed>100)speed=100;
	Right_Wheel_Speed = speed;
}

int8_t Get_LeftWheel_Speed(void)
{
	return Left_Wheel_Speed;
}

int8_t Get_RightWheel_Speed(void)
{
	return Right_Wheel_Speed;
}

void Work_Motor_Configure(void)
{
	int vaccum_pwr = 60;
	vaccum_pwr = vaccum_pwr > 0 ? vaccum_pwr : 0;
	vaccum_pwr = vaccum_pwr < 100 ? vaccum_pwr : 100;
	control_set(CTL_VACCUM_PWR, ((int)(((float)vaccum_pwr) * 2.55)) & 0xff);

	int brush_left = 30;
	control_set(CTL_BRUSH_LEFT, brush_left & 0xff);

	int brush_right = 30;
	control_set(CTL_BRUSH_RIGHT, brush_right & 0xff);

	int brush_main = 30;
	control_set(CTL_BRUSH_MAIN, brush_main & 0xff);

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
	if (robot::instance()->robot_get_battery_voltage() < LOW_BATTERY_GO_HOME_VOLTAGE){
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
	// Set the mode of vaccum.
	// The data should be Vac_Speed_Max/Vac_Speed_Normal/Vac_Speed_NormalL
	Vac_Mode = data;
}

void Set_BLDC_Speed(uint32_t S)
{
	// Set the power of BLDC
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
			Set_BLDC_Speed(Vac_Speed_Normal);
			// If work time less than 2 hours, the BLDC should be in normal level, but if more than 2 hours, it should slow down a little bit.
			//if (Get_WorkTime() < Two_Hours){
			//	Set_BLDC_Speed(Vac_Speed_Normal);
			//}else{
			//	Set_BLDC_Speed(Vac_Speed_NormalL);
			//}
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
	// Return the vaccum mode
	return Vac_Mode;
}

void Switch_VacMode(void)
{
	// Switch the vaccum mode between Max and Normal
	if (Get_VacMode() == Vac_Normal){
		Set_VacMode(Vac_Max);
	}else{
		Set_VacMode(Vac_Normal);
	}
	// Process the vaccum mode
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

uint32_t Get_Rcon_status(void)
{
	return Rcon_Status;
}

uint32_t Get_Rcon_Remote(void)
{
	return (uint32_t)ir_cmd;
}

void Set_Rcon_Remote(void)
{
	ir_cmd = robot::instance()->robot_get_ir_ctrl();
}

void Reset_Rcon_Remote(void)
{
	ir_cmd = 0;
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
	Vacuum_Voltage = Vacuum_Voltage;
	Side_Brush = Side_Brush;
	Main_Brush = Main_Brush;
	return 0;
}

void Reset_WorkTimer(void)
{
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
	// Set left and right brush PWM
	int pwm = PWM;
	control_set(CTL_BRUSH_MAIN, pwm & 0xff);
}

void Set_SideBrush_PWM(uint16_t L, uint16_t R)
{
	// Set left and right brush PWM
	int brush_left = L;
	control_set(CTL_BRUSH_LEFT, brush_left & 0xff);

	int brush_right = R;
	control_set(CTL_BRUSH_RIGHT, brush_right & 0xff);
}

void Set_Vacuum_PWM(uint8_t vacuum_pwr)
{
	vacuum_pwr = vacuum_pwr > 100 ? 100 : vacuum_pwr;
	control_set(CTL_VACCUM_PWR, ((int)(((float)vacuum_pwr) * 2.55)) & 0xff);
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
	Set_Rcon_Remote();
	switch(ir_cmd){
		case 0x80:
			if(key == Remote_Forward)
				return 1;
			break;
		case 0x40:
			if(key == Remote_Left)
				return 1;
			break;
		case 0x20:
			if(key == Remote_Right)
				return 1;
			break;
		case 0x10:
			if(key == Remote_Max)
				return 1;
			break;
		case 0x08:
			if(key == Remote_Clean)
				return 1;
			break;
		case 0x04:
			if(key == Remote_Home)
				return 1;
			break;
		case 0x02:
			if(key == Remote_Random)
				return 1;
			break;
		case 0x01:
			if(key == Remote_Spot)
				return 1;
			break;
		default:
			return 0;
	}
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
	if (key_or_clean_button_detected){
		key_or_clean_button_detected = false;
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
	// Silence_Time_Count means how many loops of sendStream loop will it be silence in one speaker sound loop
	robotbase_speaker_silence_time_count = Silence_Time_Count;
	// Trigger the update flag to start the new beep action
	robotbase_beep_update_flag = true;
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
	// Stop the vaccum, directly stop the BLDC
	Set_BLDC_Speed(0);
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


void Set_CleanTool_Power(uint8_t vaccum_val,uint8_t left_brush_val,uint8_t right_brush_val,uint8_t main_brush_val)
{
	int vaccum_pwr = vaccum_val;
	vaccum_pwr = vaccum_pwr > 0 ? vaccum_pwr : 0;
	vaccum_pwr = vaccum_pwr < 100 ? vaccum_pwr : 100;
	control_set(CTL_VACCUM_PWR, ((int)(((float)vaccum_pwr) * 2.55)) & 0xff);

	int brush_left = left_brush_val;
	control_set(CTL_BRUSH_LEFT, brush_left & 0xff);

	int brush_right = right_brush_val;
	control_set(CTL_BRUSH_RIGHT, brush_right & 0xff);

	int brush_main = main_brush_val;
	control_set(CTL_BRUSH_MAIN, brush_main & 0xff);
}

void control_set(uint8_t type, uint8_t val)
{
	if (type >= CTL_WHEEL_LEFT_HIGH && type <= CTL_GYRO) {
		sendStream[type] = val;
		//sendStream[SEND_LEN-3] = calcBufCrc8((char *)sendStream, SEND_LEN-3);
		//serial_write(SEND_LEN, sendStream);
	}
}

void control_append_crc(){
	sendStream[CTL_CRC] = calcBufCrc8((char *)sendStream, SEND_LEN-3);	
}

void control_stop_all(void)
{
	uint8_t i;

	for(i = 2; i < (SEND_LEN)-2; i++) {
		if (i == CTL_MAIN_PWR)
			sendStream[i] = 0x01;
		else
			sendStream[i] = 0x00;
	}
	//sendStream[SEND_LEN-3] = calcBufCrc8((char *)sendStream, SEND_LEN-3);
	//serial_write(SEND_LEN, sendStream);
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

void Reset_RightWheel_Step(){}
void Reset_LeftWheel_Step(){}

uint16_t GetBatteryVoltage()
{
	uint8_t i;
	uint32_t temp_v=0;	
	for(i=0;i<10;i++)
	{
		temp_v += robot::instance()->robot_get_battery_voltage();
		usleep(10000);
	}
	return (uint16_t)temp_v/10;
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
}

uint8_t Get_Key_Time(uint16_t key)
{
	uint8_t time;
	while(ros::ok()){
		time++;
		if(time>200)break;
		usleep(10000);
		if(Get_Key_Press()!=key)break;
	}
	return time;
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
