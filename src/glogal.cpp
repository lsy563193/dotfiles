//
// Created by lsy563193 on 12/14/17.
//
#include "ros/ros.h"
#include <cstdint>
#include <mathematics.h>
#include <deque>
#include <clean_mode.h>
#include <pp.h>
#include <error.h>
Cell_t g_zero_home;
bool	g_start_point_seen_charger;
bool g_have_seen_charger;
std::deque<Cell_t> g_passed_path;
bool g_move_back_finished;
static uint8_t g_cleaning_mode = 0;
bool line_is_found;
bool g_motion_init_succeeded;
bool g_resume_cleaning;
//boost::shared_ptr<State> ACleanMode::sp_state_ = nullptr;
//boost::shared_ptr<IMoveType> ACleanMode::sp_move_type_ = nullptr;
uint32_t g_wf_start_timer;
uint32_t g_wf_diff_timer;
int g_wf_reach_count;
//SpotMovement::instance()
double g_time_straight;
int g_wall_distance;
int16_t g_turn_angle;
bool g_allow_check_path_in_advance;
bool g_check_path_in_advance;
bool g_slip_backward;
time_t last_time_remote_spot = time(NULL);
std::vector<Cell_t> g_homes;
std::vector<int> g_home_way_list;
std::vector<int>::iterator g_home_way_it;
bool g_go_home_by_remote = false;
Cell_t g_home_point;

deque <Cell_t> path_points;
double bumper_turn_factor = 0.85;
CM_EventHandle eh;

void path_display_path_points(const std::deque<Cell_t>& path)
{
	std::string     msg = __FUNCTION__;

	msg += " " + std::to_string(__LINE__) + ": Path size(" + std::to_string(path.size()) + "):";
	for (auto it = path.begin(); it != path.end(); ++it) {
		msg += "(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ", " + std::to_string(it->TH) + ")->";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}
bool fw_is_time_up()
{
	return ((uint32_t)difftime(time(NULL), g_wf_start_timer)) > g_wf_diff_timer;
}

bool cm_is_exploration()
{
	return  g_cleaning_mode == Clean_Mode_Exploration;
}

bool cm_is_navigation()
{
	return g_cleaning_mode == Clean_Mode_Navigation;
}

bool cm_is_follow_wall()
{
	return g_cleaning_mode == Clean_Mode_WallFollow;
}

void cm_set(uint8_t mode)
{
	g_cleaning_mode = mode;
}

bool cm_is_go_charger()
{
	return g_cleaning_mode == Clean_Mode_Go_Charger;
}

uint8_t cm_get()
{
	return g_cleaning_mode;
}


void cm_register_events()
{
	event_manager_register_handler(&eh);
	event_manager_set_enable(true);
}


void CM_EventHandle::bumperAll(bool state_now, bool state_last)
{
//	ev.bumper_triggered = BLOCK_ALL;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, bumper.get_status(), state_now ? "true" : "false", state_last ? "true" : "false");

}

void CM_EventHandle::bumperLeft(bool state_now, bool state_last)
{
//	if(ev.bumper_triggered != 0)
//		return;
//	ev.bumper_triggered = BLOCK_LEFT;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, bumper.get_status(), state_now ? "true" : "false", state_last ? "true" : "false");
}

void CM_EventHandle::bumperRight(bool state_now, bool state_last)
{
//	if(ev.bumper_triggered != 0)
//		return;
//
//	ev.bumper_triggered = BLOCK_RIGHT;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, bumper.get_status(), state_now ? "true" : "false", state_last ? "true" : "false");
}

/* OBS */
void CM_EventHandle::obsFront(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
//	ev.obs_triggered = Status_Front_OBS;
}

void CM_EventHandle::obsLeft(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
//	ev.obs_triggered = Status_Left_OBS;
}

void CM_EventHandle::obsRight(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
//	ev.obs_triggered = Status_Right_OBS;
}

/* Cliff */
void CM_EventHandle::cliffAll(bool state_now, bool state_last)
{
	g_cliff_all_cnt++;
	if (g_cliff_all_cnt++ > 2)
	{
		ev.cliff_all_triggered = true;
		ev.fatal_quit = true;
	}
	ev.cliff_triggered = BLOCK_ALL;
	if (g_move_back_finished && !ev.cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void CM_EventHandle::cliffFrontLeft(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_LF;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");

}

void CM_EventHandle::cliffFrontRight(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_RF;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");

}

void CM_EventHandle::cliffLeftRight(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_LR;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}
//
//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void CM_EventHandle::cliffFront(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_Front;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}
//
//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void CM_EventHandle::cliffLeft(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_Left;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void CM_EventHandle::cliffRight(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_Right;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

/* RCON */
void CM_EventHandle::rcon(bool state_now, bool state_last)
{
/*
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (cs.is_going_home()) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}
	if(mt.is_follow_wall()){
		if (!(get_rcon_status() & (RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)))
			return;
	}
	else if (mt.is_linear())
		// Since we have front left 2 and front right 2 rcon receiver, seems it is not necessary to handle left or right rcon receives home signal.
		if (!(c_rcon.getStatus() & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)))
		return;

	ev.rcon_triggered = c_rcon.get_trig_();
	if(ev.rcon_triggered != 0){
		nav_map.set_rcon();
	}
	c_rcon.resetStatus();*/
}

/* Over Current */
//void CM_EventHandle::over_current_brush.left(bool state_now, bool state_last)
//{
//	static uint8_t stop_cnt = 0;
//
//	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
//	if(!robot::instance()->getLbrushOc()) {
//		g_oc_brush.left_cnt = 0;
//		if (stop_cnt++ > 250) {
//			brush.setLeftPwm(30);
//		}
//		return;
//	}
//
//	stop_cnt = 0;
//	if (g_oc_brush.left_cnt++ > 40) {
//		g_oc_brush.left_cnt = 0;
//		brush.setLeftPwm(0);
//		ROS_WARN("%s %d: left brush over current", __FUNCTION__, __LINE__);
//	}
//}

void CM_EventHandle::overCurrentBrushMain(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!brush.getMainOc()){
		brush.oc_main_cnt_ = 0;
		return;
	}

	if (brush.oc_main_cnt_++ > 40) {
		brush.oc_main_cnt_ = 0;
		ROS_WARN("%s %d: main brush over current", __FUNCTION__, __LINE__);

		if (cs_self_check(Check_Main_Brush) == 1) {
			ev.oc_brush_main = true;
			ev.fatal_quit = true;
		}
    }
}

//void CM_EventHandle::over_current_brush.right(bool state_now, bool state_last)
//{
//	static uint8_t stop_cnt = 0;
//
//	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
//
//	if(!robot::instance()->getRbrushOc()) {
//		g_oc_brush.right_cnt = 0;
//		if (stop_cnt++ > 250) {
//			brush.setRightPwm(30);
//		}
//		return;
//	}
//
//	stop_cnt = 0;
//	if (g_oc_brush.right_cnt++ > 40) {
//		g_oc_brush.right_cnt = 0;
//		brush.setRightPwm(0);
//		ROS_WARN("%s %d: reft brush over current", __FUNCTION__, __LINE__);
//	}
//}

void CM_EventHandle::overCurrentWheelLeft(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) wheel.getLeftWheelCurrent() < Wheel_Stall_Limit) {
        g_oc_wheel_left_cnt = 0;
		return;
	}

	if (g_oc_wheel_left_cnt++ > 40){
		g_oc_wheel_left_cnt = 0;
		ROS_WARN("%s %d: left wheel over current, \033[1m%u mA\033[0m", __FUNCTION__, __LINE__, (uint32_t) wheel.getLeftWheelCurrent());

		ev.oc_wheel_left = true;
	}
}

void CM_EventHandle::overCurrentWheelRight(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) wheel.getRightWheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_right_cnt = 0;
		return;
	}

	if (g_oc_wheel_right_cnt++ > 40){
		g_oc_wheel_right_cnt = 0;
		ROS_WARN("%s %d: right wheel over current, \033[1m%u mA\033[0m", __FUNCTION__, __LINE__, (uint32_t) wheel.getRightWheelCurrent());

		ev.oc_wheel_right = true;
	}
}

void CM_EventHandle::overCurrentSuction(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!vacuum.getOc()) {
		g_oc_suction_cnt = 0;
		return;
	}

	if (g_oc_suction_cnt++ > 40) {
		g_oc_suction_cnt = 0;
		ROS_WARN("%s %d: vacuum over current", __FUNCTION__, __LINE__);

		ev.oc_suction = true;
	}
}

/* Key */
void CM_EventHandle::keyClean(bool state_now, bool state_last)
{
	time_t start_time;
	bool reset_manual_pause = false;

	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (ev.slam_error)
	{
		beeper.play_for_command(INVALID);
		while (key.getPressStatus())
		{
			usleep(20000);
		}
		ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
		key.resetTriggerStatus();
		return;
	}

	beeper.play_for_command(VALID);
	wheel.stop();
	ev.key_clean_pressed = true;

	if(cm_is_navigation())
		g_is_manual_pause = true;

	start_time = time(NULL);
	while (key.getPressStatus())
	{
		if (cm_is_navigation() && time(NULL) - start_time > 3) {
			if (!reset_manual_pause)
			{
				beeper.play_for_command(VALID);
				reset_manual_pause = true;
				g_is_manual_pause = false;
				ROS_WARN("%s %d: Manual pause has been reset.", __FUNCTION__, __LINE__);
			}
		}
		else
			ROS_DEBUG("%s %d: Key clean is not released.", __FUNCTION__, __LINE__);
		usleep(20000);
	}

	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
	key.resetTriggerStatus();
}

/* Remote */

void CM_EventHandle::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (ev.slam_error)
	{
		beeper.play_for_command(INVALID);
		remote.reset();
		return;
	}
	beeper.play_for_command(VALID);
	ev.key_clean_pressed = true;
	if(cm_is_navigation()){
		g_is_manual_pause = true;
	}
	remote.reset();
}

void CM_EventHandle::remoteHome(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (/*g_motion_init_succeeded && !cs.is_going_home() && */!cm_should_self_check() && !ev.slam_error && !g_is_manual_pause) {

		/*if( SpotMovement::instance()->getSpotType()  == NORMAL_SPOT){
			beeper.play_for_command(INVALID);
		}
		else*/{
			ev.remote_home = true;
			beeper.play_for_command(VALID);
//			if (SpotMovement::instance()->getSpotType() == CLEAN_SPOT){
//				SpotMovement::instance()->spotDeinit();
//			}
		}
		ROS_INFO("ev.remote_home = %d", ev.remote_home);
	}
	else {
		beeper.play_for_command(INVALID);
		ROS_INFO("ev.remote_home = %d", ev.remote_home);
	}
	remote.reset();
}

void CM_EventHandle::remoteSpot(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!g_motion_init_succeeded || !cm_is_navigation()
		|| /*cs.is_going_home() ||*/ cm_should_self_check() || ev.slam_error || g_is_manual_pause
		|| time(NULL) - last_time_remote_spot < 3)
		beeper.play_for_command(INVALID);
	else
	{
		ev.remote_spot = true;
		last_time_remote_spot = time(NULL);
		beeper.play_for_command(VALID);
	}

	remote.reset();
}

void CM_EventHandle::remoteMax(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

//	if (g_motion_init_succeeded && /*!cs.is_going_home() && */!cm_should_self_check() && SpotMovement::instance()->getSpotType() == NO_SPOT && !ev.slam_error && !g_is_manual_pause)
//	{
//		beeper.play_for_command(VALID);
//		vacuum.switchToNext(true);
//	}
//	else
		beeper.play_for_command(INVALID);
	remote.reset();
}

/*
void CM_EventHandle::remote_direction(bool state_now,bool state_last)
{
	ROS_WARN("%s,%d: is called.",__FUNCTION__,__LINE__);
	// For Debug
	// ev.battrey_home = true;
	// path_set_continue_cell(nav_map.get_curr_cell());
	// robot::instance()->setLowBatPause();
	beeper.play_for_command(INVALID);
	remote.reset();
}

void CM_EventHandle::remote_direction_left(bool state_now, bool state_last)
{
	remote_direction(state_now,state_last);
}
void CM_EventHandle::remote_direction_right(bool state_now, bool state_last){
	remote_direction(state_now,state_last);
}

void CM_EventHandle::remote_direction_forward(bool state_now, bool state_last){
	remote_direction(state_now,state_last);
}
*/
/* Battery */
void CM_EventHandle::batteryHome(bool state_now, bool state_last)
{
	if (g_motion_init_succeeded /*&& !cs.is_going_home()*/) {
		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m", __FUNCTION__, __LINE__,
						 battery.getVoltage());
		ev.battrey_home = true;

		if (vacuum.getMode() == Vac_Max) {
			vacuum.switchToNext(false);
		}
/*#if CONTINUE_CLEANING_AFTER_CHARGE
		if (SpotMovement::instance()->getSpotType() != NORMAL_SPOT ){
			path_set_continue_cell(nav_map.getCurrCell());
			g_is_low_bat_pause = true;
		}
#endif
 */
    }
}

void CM_EventHandle::batteryLow(bool state_now, bool state_last)
{
    uint8_t         v_pwr, s_pwr, m_pwr;
    uint16_t        t_vol;

    ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_battery_low_cnt++ > 50) {
		t_vol = battery.getVoltage();
		ROS_WARN("%s %d: low battery, battery < %umv is detected.", __FUNCTION__, __LINE__,t_vol);

/*		if (cs.is_going_home()) {
			v_pwr = Home_Vac_Power / t_vol;
			s_pwr = Home_SideBrush_Power / t_vol;
			m_pwr = Home_MainBrush_Power / t_vol;
		} else {
			v_pwr = Clean_Vac_Power / t_vol;
			s_pwr = Clean_SideBrush_Power / t_vol;
			m_pwr = Clean_MainBrush_Power / t_vol;
		}*/

		g_battery_low_cnt = 0;
		vacuum.bldcSpeed(v_pwr);
		brush.setSidePwm(s_pwr, s_pwr);
		brush.setMainPwm(m_pwr);

		ev.fatal_quit = true;
		ev.battery_low = true;
	}
}

void CM_EventHandle::chargeDetect(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: Detect charger: %d, g_charge_detect_cnt: %d.", __FUNCTION__, __LINE__, charger.getChargeStatus(), g_charge_detect_cnt);
	if (((cm_is_exploration() || cm_is_go_charger() /*|| cs.is_going_home()*/) && charger.getChargeStatus()) ||
		(!cm_is_exploration() && charger.getChargeStatus() == 3))
	{
		if (g_charge_detect_cnt++ > 2)
		{
			ev.charge_detect = charger.getChargeStatus();
			if (!cm_is_exploration() && charger.getChargeStatus() == 3)
				ev.fatal_quit = true;
			ROS_WARN("%s %d: ev.charge_detect has been set to %d.", __FUNCTION__, __LINE__, ev.charge_detect);
			g_charge_detect_cnt = 0;
		}
	}
}
bool is_fobbit_free() {
	//NOTE: g_home_way_it should last of g_home_point,for g_homeway_list may empty.
	return (/*cs.is_going_home() &&*/ *g_home_way_it % HOMEWAY_NUM == USE_CLEANED);
}


bool cm_should_self_check(void)
{
	return (ev.oc_wheel_left || ev.oc_wheel_right || ev.bumper_jam || ev.cliff_jam || ev.oc_suction || g_slip_cnt >= 2 || ev.lidar_stuck);
}

void path_set_home(const Cell_t& curr)
{
	bool is_found = false;

	for (const auto& it : g_homes) {
		ROS_INFO("%s %d: curr\033[33m(%d, %d)\033[0m home_it\033[33m(%d,%d)\033[0m.", __FUNCTION__, __LINE__, curr.X, curr.Y,it.X,it.Y);
		if (it == curr) {
			is_found = true;
			break;
		}
	}
	if (!is_found) {
		ROS_INFO("%s %d: Push new reachable home:\033[33m (%d, %d)\033[0m to home point list.", __FUNCTION__, __LINE__, curr.X, curr.Y);
		if(cm_get() != Clean_Mode_Spot)
			g_have_seen_charger = true;
		// If curr near (0, 0)
		if (abs(curr.X) >= 5 || abs(curr.Y) >= 5)
		{
			if(g_homes.size() >= ESCAPE_TRAPPED_REF_CELL_SIZE+1)//escape_count + zero_home = 3+1 = 4
			{
				std::copy(g_homes.begin() + 2, g_homes.end(), g_homes.begin()+1);//shift 1 but save zero_home
				g_homes.pop_back();
			}
			g_homes.push_back(curr);
		}
	}
	else if(curr == g_zero_home && cm_get() != Clean_Mode_Spot)
	{
		g_start_point_seen_charger = true;
		g_have_seen_charger = true;
	}
}

uint8_t cs_self_check(uint8_t Check_Code)
{
	static time_t mboctime;
	static time_t vacoctime;
	static uint8_t mbrushchecking = 0;
	uint8_t Time_Out = 0;
	int32_t Wheel_Current_Summary = 0;
	uint8_t Left_Wheel_Slow = 0;
	uint8_t Right_Wheel_Slow = 0;

/*
	if(cm_is_navigation())
		cm_move_back_(COR_BACK_20MM);
	else
		quick_back(30,20);
*/
	cs_disable_motors();
	usleep(10000);
	/*------------------------------Self Check right wheel -------------------*/
	if (Check_Code == Check_Right_Wheel)
	{
		Right_Wheel_Slow = 0;
		if (wheel.getDirection() == DIRECTION_LEFT)
		{
			wheel.setDirectionRight();
		} else
		{
			wheel.setDirectionLeft();
		}
		wheel.setPidTargetSpeed(30, 30);
		usleep(50000);
		Time_Out = 50;
		Wheel_Current_Summary = 0;
		while (Time_Out--)
		{
			Wheel_Current_Summary += (uint32_t) wheel.getRightWheelCurrent();
			usleep(20000);
		}
		Wheel_Current_Summary /= 50;
		if (Wheel_Current_Summary > Wheel_Stall_Limit)
		{
			cs_disable_motors();
			ROS_WARN("%s,%d right wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
			error.set(ERROR_CODE_RIGHTWHEEL);
			error.alarm();
			return 1;

		}
		/*
		if(Right_Wheel_Slow>100)
		{
			cs_disable_motors();
			error.set(Error_Code_RightWheel);
			return 1;
		}
		*/
		wheel.stop();
		//turn_right(Turn_Speed,1800);
	}
		/*---------------------------Self Check left wheel -------------------*/
	else if (Check_Code == Check_Left_Wheel)
	{
		Left_Wheel_Slow = 0;
		if (wheel.getDirection() == DIRECTION_RIGHT)
		{
			wheel.setDirectionLeft();
		} else
		{
			wheel.setDirectionRight();
		}
		wheel.setPidTargetSpeed(30, 30);
		usleep(50000);
		Time_Out = 50;
		Wheel_Current_Summary = 0;
		while (Time_Out--)
		{
			Wheel_Current_Summary += (uint32_t) wheel.getLeftWheelCurrent();
			usleep(20000);
		}
		Wheel_Current_Summary /= 50;
		if (Wheel_Current_Summary > Wheel_Stall_Limit)
		{
			cs_disable_motors();
			ROS_WARN("%s %d,left wheel stall maybe, please check!!", __FUNCTION__, __LINE__);
			error.set(ERROR_CODE_LEFTWHEEL);
			error.alarm();
			return 1;
		}
		/*
		if(Left_Wheel_Slow>100)
		{
			cs_disable_motors();
			error.set(ERROR_CODE_RIGHTWHEEL);
			return 1;
		}
		*/
		wheel.stop();
		//turn_left(Turn_Speed,1800);
	} else if (Check_Code == Check_Main_Brush)
	{
		if (!mbrushchecking)
		{
			brush.setMainPwm(0);
			mbrushchecking = 1;
			mboctime = time(NULL);
		} else if ((uint32_t) difftime(time(NULL), mboctime) >= 3)
		{
			mbrushchecking = 0;
			error.set(ERROR_CODE_MAINBRUSH);
			cs_disable_motors();
			error.alarm();
			return 1;
		}
		return 0;
	} else if (Check_Code == Check_Vacuum)
	{
#ifndef BLDC_INSTALL
		ROS_INFO("%s, %d: Vacuum Over Current!!", __FUNCTION__, __LINE__);
		ROS_INFO("%d", vacuum.getSelfCheckStatus());
		while (vacuum.getSelfCheckStatus() != 0x10)
		{
			/*-----wait until self check begin-----*/
			vacuum.startSelfCheck();
		}
		ROS_INFO("%s, %d: Vacuum Self checking", __FUNCTION__, __LINE__);
		/*-----reset command for start self check-----*/
		vacuum.resetSelfCheck();
		/*-----wait for the end of self check-----*/
		while (vacuum.getSelfCheckStatus() == 0x10);
		ROS_INFO("%s, %d: end of Self checking", __FUNCTION__, __LINE__);
		if (vacuum.getSelfCheckStatus() == 0x20)
		{
			ROS_INFO("%s, %d: Vacuum error", __FUNCTION__, __LINE__);
			/*-----vacuum error-----*/
			error.set(ERROR_CODE_FAN_H);
			cs_disable_motors();
			error.alarm();
			vacuum.resetSelfCheck();
			return 1;
		}
		vacuum.resetSelfCheck();
#else
		Disable_Motors();
		//wheel.stop();
		Set_Vac_Speed();
		usleep(100000);
		vacoctime = time(NULL);
		uint16_t tmpnoc_n = 0;
		while((uint32_t)difftime(time(NULL),vacoctime)<=3){
			if(!robot::instance()->robot_get_vacuum_oc()){
				tmpnoc_n++;
				if(tmpnoc_n>20){
					Work_Motor_Configure();
					tmpnoc_n = 0;
					return 0;
				}
			}
			usleep(50000);
		}
		error.set(Error_Code_Fan_H);
		cs_disable_motors();
		Alarm_Error();
		return 1;
#endif
	} else if (Check_Code == Check_Left_Brush)
	{
		error.set(ERROR_CODE_LEFTBRUSH);
		cs_disable_motors();
		error.alarm();
		return 1;
	} else if (Check_Code == Check_Right_Brush)
	{
		error.set(ERROR_CODE_RIGHTBRUSH);
		cs_disable_motors();
		error.alarm();
		return 1;
	}
	wheel.stop();
	Left_Wheel_Slow = 0;
	Right_Wheel_Slow = 0;
	cs_work_motor(false);
	//wheel.moveForward(5,5);
	return 0;
}

void cs_disable_motors(void)
{
	wheel.stop();
	brush.stop();
	vacuum.stop();
}


void cs_work_motor(bool st_is_go_home)
{
	if (st_is_go_home)
	{
		// Set the vacuum to a normal mode
		vacuum.setMode(Vac_Normal, false);
		// Turn on the main brush and side brush
		brush.setSidePwm(30, 30);
	} else {
		vacuum.setMode(Vac_Save);
		// Turn on the main brush and side brush
		brush.setSidePwm(50, 50);
	}

	brush.setMainPwm(30);
}


bool check_pub_scan()
{
	//ROS_INFO("%s %d: get_left_wheel.speed() = %d, get_right_wheel.speed() = %d.", __FUNCTION__, __LINE__, wheel.getLeftSpeedAfterPid(), wheel.getRightSpeedAfterPid());
	if (g_motion_init_succeeded &&
		((fabs(wheel.getLeftWheelActualSpeed() - wheel.getRightWheelActualSpeed()) > 0.1)
		|| (wheel.getLeftWheelActualSpeed() * wheel.getRightWheelActualSpeed() < 0)
		|| bumper.get_status() || gyro.getTiltCheckingStatus()
		|| abs(wheel.getLeftSpeedAfterPid() - wheel.getRightSpeedAfterPid()) > 100
		|| wheel.getLeftSpeedAfterPid() * wheel.getRightSpeedAfterPid() < 0))
		return false;
	else
		return true;
}

