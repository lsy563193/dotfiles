#include "pp.h"

#ifdef TURN_SPEED
#undef TURN_SPEED
#endif

#define TURN_SPEED	17
/*
 * MOVE_TO_CELL_SEARCH_INCREMENT: offset from robot's current position.
 * MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH: max searching radius from robot's current position.
 */
#define MOVE_TO_CELL_SEARCH_INCREMENT 2
#define MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH 10
#define MOVE_TO_CELL_SEARCH_ARRAY_LENGTH (2 * MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH / MOVE_TO_CELL_SEARCH_INCREMENT + 1)
#define MOVE_TO_CELL_SEARCH_ARRAY_LENGTH_MID_IDX ((MOVE_TO_CELL_SEARCH_ARRAY_LENGTH * MOVE_TO_CELL_SEARCH_ARRAY_LENGTH - 1) / 2)

#define	MAX_SPEED		(30)
#define WHEEL_BASE_HALF			(((double)(WHEEL_BASE)) / 2)
#define CELL_COUNT	(((double) (CELL_COUNT_MUL)) / CELL_SIZE)
#define RADIUS_CELL (3 * CELL_COUNT_MUL)

uint16_t g_straight_distance;
PPTargetType g_plan_path;
std::deque<Cell_t> g_passed_path;
bool g_exploration_home;
bool g_rcon_during_go_home;
//std::vector<int16_t> g_left_buffer;
//std::vector<int16_t> g_right_buffer;

//Cell_t g_next_cell;
//Cell_t g_target_cell;

//uint8_t	g_remote_go_home = 0;
//bool	cs_is_going_home() = false;
bool	g_from_station = 0;
bool	g_in_charge_signal_range;

int16_t g_map_gyro_offset = 0;
// This flag is indicating robot is resuming from low battery go home.
bool g_resume_cleaning = false;

// This flag is for checking whether map boundary is created.

bool g_have_seen_charger = false;
bool g_start_point_seen_charger = false;

Cell_t g_relativePos[MOVE_TO_CELL_SEARCH_ARRAY_LENGTH * MOVE_TO_CELL_SEARCH_ARRAY_LENGTH] = {{0, 0}};

long g_distance=0;
bool g_is_near=false;
extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

// This flag is for indicating robot is going to charger.
bool g_during_go_to_charger = false;

// Saved position for move back.
float saved_pos_x, saved_pos_y;

// Flag for indicating whether move back has finished.
bool g_move_back_finished = true;

// Flag for indicating whether motion instance is initialized successfully.
bool g_motion_init_succeeded = false;

bool g_go_home_by_remote = false;

//Flag for judge if keep on wall follow
bool g_keep_on_wf;

bool g_finish_cleaning_go_home = false;

bool g_no_uncleaned_target = false;

time_t last_time_remote_spot = time(NULL);
int16_t ranged_angle(int16_t angle)
{
	while (angle > 1800 || angle <= -1800)
	{
		if (angle > 1800) {
			angle -= 3600;
		} else
		if (angle <= -1800) {
			angle += 3600;
		}
	}
	return angle;
}

bool is_map_front_block(int dx)
{
	int32_t x, y;
	for (auto dy = -1; dy <= 1; dy++)
	{
		cm_world_to_point(gyro_get_angle(), dy * CELL_SIZE, CELL_SIZE * dx, &x, &y);
		if (map_get_cell(MAP, count_to_cell(x), count_to_cell(y)) == BLOCKED_BOUNDARY)
			return true;
	}
	return false;
}

//--------------------------------------------------------

void cm_world_to_point_accurate(int16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y)
{
	*x = map_get_relative_x(heading, offset_lat, offset_long, true);
	*y = map_get_relative_y(heading, offset_lat, offset_long, true);
}
void cm_world_to_point(int16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y)
{
	*x = cell_to_count(count_to_cell(map_get_relative_x(heading, offset_lat, offset_long, true)));
	*y = cell_to_count(count_to_cell(map_get_relative_y(heading, offset_lat, offset_long, true)));
}

void cm_world_to_cell(int16_t heading, int16_t offset_lat, int16_t offset_long, int16_t &x, int16_t &y)
{
	x = count_to_cell(map_get_relative_x(heading, offset_lat, offset_long, false));
	y = count_to_cell(map_get_relative_y(heading, offset_lat, offset_long, false));
}

int cm_get_grid_index(float position_x, float position_y, uint32_t width, uint32_t height, float resolution,
											double origin_x, double origin_y)
{
	int index, grid_x, grid_y;

	/*get index*/
	grid_x = int(round((position_x - origin_x) / resolution));
	grid_y = int(round((position_y - origin_y) / resolution));
	index = grid_x + grid_y * width;
	
	return index;
}

void set_wheel_speed_pid(const CleanMode* rm,int32_t speed_left,int32_t speed_right)
{
#if GLOBAL_PID
		/*---PID is useless in wall follow mode---*/
		if(rm->isMt() && mt_is_follow_wall())
			set_wheel_speed(speed_left, speed_right, REG_TYPE_WALLFOLLOW);
		else if(rm->isMt() && mt_is_linear())
			set_wheel_speed(speed_left, speed_right, REG_TYPE_LINEAR);
		else if(rm->isMt() && mt_is_go_to_charger())
			set_wheel_speed(speed_left, speed_right, REG_TYPE_NONE);
		else if(rm->isBack())
			set_wheel_speed(speed_left, speed_right, REG_TYPE_BACK);
		else if(rm->isTurn())
			set_wheel_speed(speed_left, speed_right, REG_TYPE_TURN);
#else
		/*---PID is useless in wall follow mode---*/
		set_wheel_speed(speed_left, speed_right, REG_TYPE_NONE);
#endif
}

void cm_self_check_with_handle(void)
{
	// Can not set handler state inside cm_self_check(), because it is actually a universal function.
	cm_set_event_manager_handler_state(true);
	cm_self_check();
	if(cm_is_follow_wall()) {
		g_keep_on_wf = true;
	}
	cm_set_event_manager_handler_state(false);
}

void cm_move_to(CleanMode* p_cm, PPTargetType path) {
if (mt_is_linear()) {
		wall_dynamic_base(30);
	}

	if (p_cm->isSwitch()) {
		map_save_blocks();
		//map_set_blocks();
		//MotionManage::pubCleanMapMarkers(MAP, g_plan_path);
	}

	int32_t speed_left = 0, speed_right = 0;
	p_cm->adjustSpeed(speed_left, speed_right);
	set_wheel_speed_pid(p_cm, speed_left, speed_right);
}

void cm_cleaning() {
	MotionManage motion;
	if (!motion.initSucceeded())
		return;

	g_motion_init_succeeded = true;
	cs_init();
	Cell_t curr = map_update_position();

	CleanMode* p_cm;
	if(cm_is_follow_wall())
		p_cm = new WallFollowClean(curr, g_plan_path.front(), g_plan_path);
	else if(cm_is_exploration())
		p_cm = new Exploration(curr, g_plan_path.front(), g_plan_path);
	else if(cm_is_spot())
		p_cm = new SpotClean(curr, g_plan_path.front(), g_plan_path);
	else
		p_cm = new NavigationClean(curr, g_plan_path.front(), g_plan_path);

	bool eh_status_now = false, eh_status_last = false;

	while (ros::ok()) {

		if (p_cm->isExit()) {
			break;
		}

		if (ev.slam_error) {
			set_wheel_speed(0, 0);
			continue;
		}

		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		curr = p_cm->updatePosition({map_get_x_count(), map_get_y_count()});

		if (p_cm->isReach() || p_cm->isStop())
		{
			if(!p_cm->findTarget(curr))
				return;
		}

		cm_move_to(p_cm, g_plan_path);

	if (cm_should_self_check())
		cm_self_check_with_handle();
	}
}

void cs_setting(int cs) {
	if(cs == CS_GO_HOME_POINT) {
		work_motor_configure();
		set_wheel_speed(0, 0, REG_TYPE_LINEAR);
		if (ev.remote_home || cm_is_go_charger())
			set_led_mode(LED_STEADY, LED_ORANGE);

		// Special handling for wall follow mode.
		if (cm_is_follow_wall()) {
			robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle); //For wall follow mode.
			map_update_position();
			//wf_mark_home_point();
			map_reset(MAP);
			ros_map_convert(MAP, true, false, false);
			map_mark_robot(MAP);//note: To clear the obstacles before go home, please don't remove it!
		}
		// Play wavs.
		if (ev.battrey_home)
			wav_play(WAV_BATTERY_LOW);
		if (!cm_is_go_charger())
			wav_play(WAV_BACK_TO_CHARGER);

		if (ev.remote_home)
			g_go_home_by_remote = true;
		ev.remote_home = false;
		ev.battrey_home = false;
		mt_set(CM_LINEARMOVE);
	}
	if(cs == CS_TMP_SPOT)
	{
		if( SpotMovement::instance() -> getSpotType() == NO_SPOT){
			ROS_INFO("%s %d: Entering temp spot during navigation.", __FUNCTION__, __LINE__);
			Cell_t curr_cell = map_get_curr_cell();
			ROS_WARN("%s %d: current cell(%d, %d).", __FUNCTION__, __LINE__, curr_cell.X, curr_cell.Y);
			SpotMovement::instance() ->setSpotType(CLEAN_SPOT);
			set_wheel_speed(0, 0);
		}
		else if(SpotMovement::instance()->getSpotType() == CLEAN_SPOT){
			ROS_INFO("%s %d: Exiting temp spot.", __FUNCTION__, __LINE__);
			SpotMovement::instance()->spotDeinit();
			set_wheel_speed(0, 0);
			wav_play(WAV_CLEANING_CONTINUE);
		}
		ev.remote_spot = false;
	}
	if(cs == CS_TRAPPED)
	{
		g_wf_start_timer = time(NULL);
		g_wf_diff_timer = ESCAPE_TRAPPED_TIME;
		set_led_mode(LED_FLASH, LED_GREEN, 300);
		mt_set(CM_FOLLOW_LEFT_WALL);
	}
	if(cs == CS_CLEAN) {
		g_wf_reach_count = 0;
		set_led_mode(LED_STEADY, LED_GREEN);
	}
	if(cs == CS_EXPLORATION) {
		mt_set(CM_LINEARMOVE);
		g_wf_reach_count = 0;
		set_led_mode(LED_STEADY, LED_ORANGE);
	}
	if(cs == CS_GO_CHANGER)
	{
		g_tilt_enable = false; //disable tilt detect
		set_led_mode(LED_STEADY, LED_ORANGE);
	}
}
/* Statement for cm_go_to_charger_(void)
 * return : true -- going to charger has been stopped, either successfully or interrupted.
 *          false -- going to charger failed, move to next point.
 */
bool cm_go_to_charger()
{
	// Try to go to charger stub.
	ROS_INFO("%s %d: Try to go to charger stub,\033[35m disable tilt detect\033[0m.", __FUNCTION__, __LINE__);
	g_tilt_enable = false; //disable tilt detect
	set_led_mode(LED_STEADY, LED_ORANGE);
	mt_set(CM_GO_TO_CHARGER);
//	PPTargetType path_empty;
//	set_led_mode(LED_STEADY, LED_GREEN);
//
//	if (ev.fatal_quit || ev.key_clean_pressed || ev.charge_detect)
//		return true;
//	work_motor_configure();
//	g_tilt_enable = true; //enable tilt detect
//	ROS_INFO("\033[35m" "%s,%d,enable tilt detect" "\033[0m",__FUNCTION__,__LINE__);
	return false;
}

/*bool cm_is_continue_go_to_charger()
{
	auto way = *g_home_way_it % HOMEWAY_NUM;
	auto cnt = *g_home_way_it / HOMEWAY_NUM;
	ROS_INFO("%s,%d:g_home_point(\033[1;46;37m%d,%d\033[0m), way(\033[1;46;37m%d\033[0m), cnt(\033[1;46;37m%d\033[0m) ", __FUNCTION__, __LINE__,g_home_point.X,g_home_point.Y,way, cnt);
	if(cm_go_to_charger() || cnt == 0)
	{
		ROS_INFO("\033[1;46;37m" "%s,%d:cm_go_to_charger_ stop " "\033[0m", __FUNCTION__, __LINE__);
		return false;
	}
	else if(!g_go_home_by_remote)
		set_led_mode(LED_STEADY, LED_GREEN);
	g_home_way_it += (way+1);
	g_homes.pop_back();
	g_home_way_list.clear();
	ROS_INFO("\033[1;46;37m" "%s,%d:cm_is_continue_go_to_charger_home(%d,%d), way(%d), cnt(%d) " "\033[0m", __FUNCTION__, __LINE__,g_home_point.X, g_home_point.Y,way, cnt);
	return true;
}*/

void cm_reset_go_home(void)
{
	ROS_DEBUG("%s %d: Reset go home flags here.", __FUNCTION__, __LINE__);
	cs_set(CS_CLEAN);
	g_go_home_by_remote = false;
}
/*

bool cm_check_loop_back(Cell_t target)
{
	bool retval = false;
	if ( target == g_cell_history[1] && target == g_cell_history[3]) {
		ROS_WARN("%s %d Possible loop back (%d, %d), g_cell_history:(%d, %d) (%d, %d) (%d, %d) (%d, %d) (%d, %d).", __FUNCTION__, __LINE__,
						target.X, target.Y,
						g_cell_history[0].X, g_cell_history[0].Y,
						g_cell_history[1].X, g_cell_history[1].Y,
						g_cell_history[2].X, g_cell_history[2].Y,
						g_cell_history[3].X, g_cell_history[3].Y,
						g_cell_history[4].X, g_cell_history[4].Y);
		retval	= true;
	}

	return retval;
}
*/

void cm_create_home_boundary(void)
{
	int16_t i, j, k;
	int16_t xMinSearch, xMaxSearch, yMinSearch, yMaxSearch;

	k = 3;
	xMinSearch = xMaxSearch = yMinSearch = yMaxSearch = SHRT_MAX;
	for (i = g_x_min; xMinSearch == SHRT_MAX; i++) {
		for (j = g_y_min; j <= g_y_max; j++) {
			if (map_get_cell(MAP, i, j) != UNCLEAN) {
				xMinSearch = i - k;
				break;
			}
		}
	}
	for (i = g_x_max; xMaxSearch == SHRT_MAX; i--) {
		for (j = g_y_min; j <= g_y_max; j++) {
			if (map_get_cell(MAP, i, j) != UNCLEAN) {
				xMaxSearch = i + k;
				break;
			}
		}
	}
	for (i = g_y_min; yMinSearch == SHRT_MAX; i++) {
		for (j = g_x_min; j <= g_x_max; j++) {
			if (map_get_cell(MAP, j, i) != UNCLEAN) {
				yMinSearch = i - k;
				break;
			}
		}
	}
	for (i = g_y_max; yMaxSearch == SHRT_MAX; i--) {
		for (j = g_x_min; j <= g_x_max; j++) {
			if (map_get_cell(MAP, j, i) != UNCLEAN) {
				yMaxSearch = i + k;
				break;
			}
		}
	}
	ROS_INFO("%s %d: x: %d - %d\ty: %d - %d", __FUNCTION__, __LINE__, xMinSearch, xMaxSearch, yMinSearch, yMaxSearch);
	for (i = xMinSearch; i <= xMaxSearch; i++) {
		if (i == xMinSearch || i == xMaxSearch) {
			for (j = yMinSearch; j <= yMaxSearch; j++) {
				map_set_cell(MAP, cell_to_count(i), cell_to_count(j), BLOCKED_BUMPER);
			}
		} else {
			map_set_cell(MAP, cell_to_count(i), cell_to_count(yMinSearch), BLOCKED_BUMPER);
			map_set_cell(MAP, cell_to_count(i), cell_to_count(yMaxSearch), BLOCKED_BUMPER);
		}
	}

	// Set the flag.
}

/*------------- Self check and resume-------------------*/
void cm_self_check(void)
{
	ROS_WARN("%s %d: Try to resume the oc or jam cases.", __FUNCTION__, __LINE__);
	uint8_t resume_cnt = 0;
	time_t start_time = time(NULL);
	float wheel_current_sum = 0;
	uint8_t wheel_current_sum_cnt = 0;
	uint8_t bumper_jam_state = 1;
	uint8_t vacuum_oc_state = 1;
	int16_t target_angle = 0;
	bool eh_status_now=false, eh_status_last=false;

	if (ev.bumper_jam || ev.cliff_jam || g_omni_notmove || ev.laser_stuck)
	{
		// Save current position for moving back detection.
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
	}

	if (ev.oc_wheel_left || ev.oc_wheel_right)
		disable_motors();

	if (ev.oc_suction)
	{
		ROS_WARN("%s, %d: Vacuum Self checking start", __FUNCTION__, __LINE__);
		disable_motors();
		start_self_check_vacuum();
	}

	SelfCheckRegulator regulator;

	robot::instance()->obsAdjustCount(50);
	while (ros::ok()) {
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		if (ev.fatal_quit || ev.key_clean_pressed || ev.charge_detect)
			break;

		if (ev.slam_error)
		{
			set_wheel_speed(0, 0);
			continue;
		}

		// Check for right wheel.
		if (ev.oc_wheel_left || ev.oc_wheel_right)
		{
			if (time(NULL) - start_time >= 1)
			{
				wheel_current_sum /= wheel_current_sum_cnt;
				if (wheel_current_sum > Wheel_Stall_Limit)
				{
					if (resume_cnt >= 3)
					{
						disable_motors();
						if (ev.oc_wheel_left)
						{
							ROS_WARN("%s,%d Left wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
							set_error_code(Error_Code_LeftWheel);
						}
						else
						{
							ROS_WARN("%s,%d Right wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
							set_error_code(Error_Code_RightWheel);
						}
						ev.fatal_quit = true;
						break;
					}
					else
					{
						start_time = time(NULL);
						resume_cnt++;
						wheel_current_sum = 0;
						wheel_current_sum_cnt = 0;
						ROS_WARN("%s %d: Failed to resume for %d times.", __FUNCTION__, __LINE__, resume_cnt);
					}
				}
				else
				{
					if (ev.oc_wheel_left)
					{
						ROS_WARN("%s %d: Left wheel resume succeeded.", __FUNCTION__, __LINE__);
						ev.oc_wheel_left = false;
						work_motor_configure();
					}
					else
					{
						ROS_WARN("%s %d: Left wheel resume succeeded.", __FUNCTION__, __LINE__);
						ev.oc_wheel_right = false;
						work_motor_configure();
					}
				}
			}
			else
			{
				if (ev.oc_wheel_left)
					wheel_current_sum += (uint32_t) robot::instance()->getLwheelCurrent();
				else
					wheel_current_sum += (uint32_t) robot::instance()->getRwheelCurrent();
				wheel_current_sum_cnt++;
			}
		}
		else if (ev.cliff_jam)
		{
			if (!get_cliff_status())
			{
				ROS_WARN("%s %d: Cliff resume succeeded.", __FUNCTION__, __LINE__);
				ev.cliff_triggered = 0;
				ev.cliff_all_triggered = false;
				g_cliff_cnt = 0;
				g_cliff_all_cnt = 0;
				ev.cliff_jam = false;
			}
			float distance;
			distance = sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2));
			if (fabsf(distance) > 0.05f)
			{
				if (++resume_cnt >= 3)
				{
					ROS_WARN("%s %d: Cliff jamed.", __FUNCTION__, __LINE__);
					ev.fatal_quit = true;
					set_error_code(Error_Code_Cliff);
				}
				else
				{
					stop_brifly();
					saved_pos_x = robot::instance()->getOdomPositionX();
					saved_pos_y = robot::instance()->getOdomPositionY();
				}
			}
		}
		else if (ev.bumper_jam)
		{
			if (!get_bumper_status())
			{
				ROS_WARN("%s %d: Bumper resume succeeded.", __FUNCTION__, __LINE__);
				ev.bumper_jam = false;
				ev.bumper_triggered = false;
				g_bumper_cnt = 0;
			}

			switch (bumper_jam_state)
			{
				case 1: // Move back for the first time.
				case 2: // Move back for the second time.
				{
					float distance;
					distance = sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2));
					if (fabsf(distance) > 0.05f)
					{
						stop_brifly();
						// If cliff jam during bumper self resume.
						if (get_cliff_status() && ++g_cliff_cnt > 2)
						{
							ev.cliff_jam = true;
							resume_cnt = 0;
						}
						else
						{
							bumper_jam_state++;
							ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state);
						}
						saved_pos_x = robot::instance()->getOdomPositionX();
						saved_pos_y = robot::instance()->getOdomPositionY();
					}
					break;
				}
				case 3: // Move back for the third time.
				{
					float distance;
					distance = sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2));
					if (fabsf(distance) > 0.05f)
					{
						// If cliff jam during bumper self resume.
						if (get_cliff_status() && ++g_cliff_cnt > 2)
						{
							ev.cliff_jam = true;
							resume_cnt = 0;
						}
						else
						{
							bumper_jam_state++;
							ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state);
							target_angle = ranged_angle(gyro_get_angle() - 900);
							ROS_WARN("%s %d: target_angle:%d.", __FUNCTION__, __LINE__, target_angle);
						}
					}
					break;
				}
				case 4:
				{
					ROS_DEBUG("%s %d: gyro_get_angle(): %d", __FUNCTION__, __LINE__, gyro_get_angle());
					// If cliff jam during bumper self resume.
					if (get_cliff_status() && ++g_cliff_cnt > 2)
					{
						ev.cliff_jam = true;
						resume_cnt = 0;
					}
					else if (abs(ranged_angle(gyro_get_angle() - target_angle)) < 50)
					{
						bumper_jam_state++;
						ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state);
						target_angle = ranged_angle(gyro_get_angle() + 900);
						ROS_WARN("%s %d: target_angle:%d.", __FUNCTION__, __LINE__, target_angle);
					}
					break;
				}
				case 5:
				{
					// If cliff jam during bumper self resume.
					if (get_cliff_status() && ++g_cliff_cnt > 2)
					{
						ev.cliff_jam = true;
						resume_cnt = 0;
					}
					else if (abs(ranged_angle(gyro_get_angle() - target_angle)) < 50)
					{
						ROS_WARN("%s %d: Bumper jamed.", __FUNCTION__, __LINE__);
						ev.fatal_quit = true;
						set_error_code(Error_Code_Bumper);
					}
					break;
				}
			}
		}
		else if (ev.oc_suction)
		{
			if(vacuum_oc_state == 1)
			{
				ROS_DEBUG("%s %d: Wait for suction self check begin.", __FUNCTION__, __LINE__);
				if (get_self_check_vacuum_status() == 0x10)
				{
					ROS_WARN("%s %d: Suction self check begin.", __FUNCTION__, __LINE__);
					reset_self_check_vacuum_controler();
					vacuum_oc_state = 2;
				}
				continue;
			}
			else if (vacuum_oc_state == 2)
			{
				ROS_DEBUG("%s %d: Wait for suction self check result.", __FUNCTION__, __LINE__);
				if (get_self_check_vacuum_status() == 0x20)
				{
					ROS_WARN("%s %d: Resume suction failed.", __FUNCTION__, __LINE__);
					set_error_code(Error_Code_Fan_H);
					ev.fatal_quit = true;
					break;
				}
				else if (get_self_check_vacuum_status() == 0x00)
				{
					ROS_WARN("%s %d: Resume suction succeeded.", __FUNCTION__, __LINE__);
					ev.oc_suction = false;
					g_oc_suction_cnt = 0;
					break;
				}
			}
		}
		else if (g_omni_notmove)
		{
			ROS_ERROR("\033[1m" "%s,%d,omni detect" "\033[0m",__FUNCTION__,__LINE__);
			set_error_code(Error_Code_Omni);
			ev.fatal_quit = true;
			break;
		}
		else if (g_slip_cnt >= 2)
		{
			if(g_slip_cnt < 3 && g_robot_slip){
				g_robot_slip = false;
				target_angle = ranged_angle(gyro_get_angle() + 900);	
				ROS_INFO("%s,%d,\033[32mrobot slip again slip count %d\033[0m",__FUNCTION__,__LINE__,g_slip_cnt);
			}
			else if(g_slip_cnt <4 && g_robot_slip){
				g_robot_slip = false;
				target_angle = ranged_angle(gyro_get_angle() - 900);
				ROS_INFO("%s,%d,\033[32mrobot slip again slip count %d\033[0m",__FUNCTION__,__LINE__,g_slip_cnt);
			}
			/*
			if(g_robot_slip){
				g_robot_slip = false;
				ROS_INFO("%s,%d,robot slip again",__FUNCTION__,__LINE__);
			}
			*/
			if( abs(gyro_get_angle() - target_angle) <= 50 ){
				g_slip_cnt = 0;
				g_robot_slip = false;
				ROS_INFO("\033[32m%s,%d,reach target angle\033[0m ",__FUNCTION__,__LINE__);
				break;
			}
			if(g_slip_cnt>=4){
				ROS_INFO("%s,%d,robot stuck ,slip count\033[32m %d \033[0m",__FUNCTION__,__LINE__,g_slip_cnt);
				g_slip_cnt = 0;
				g_robot_stuck = true;
				set_error_code(Error_Code_Stuck);
				//ev.fatal_quit = true;
				break;
			}
		}
		else if (ev.laser_stuck)
		{
			if (!check_laser_stuck())
			{
				ROS_WARN("%s %d: Restore from laser stuck.", __FUNCTION__, __LINE__);
				ev.laser_stuck = false;
			}

			float distance = sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2));
			if (fabsf(distance) > 0.35f)
			{
				ROS_WARN("%s %d: Laser stuck.", __FUNCTION__, __LINE__);
				ev.fatal_quit = true;
				set_error_code(Error_Code_Laser);
			}
		}
		else
			break;

		regulator.adjustSpeed(bumper_jam_state);
	}
}

bool cm_should_self_check(void)
{
	return (ev.oc_wheel_left || ev.oc_wheel_right || ev.bumper_jam || ev.cliff_jam || ev.oc_suction || g_omni_notmove || g_slip_cnt >= 2 || ev.laser_stuck);
}

/* Event handler functions. */
void cm_register_events()
{
	ROS_INFO("%s %d: Register events", __FUNCTION__, __LINE__);
	event_manager_set_current_mode(EVT_MODE_NAVIGATION);

	/* Bumper */
	event_manager_register_handler(EVT_BUMPER_ALL, &cm_handle_bumper_all);
	event_manager_register_handler(EVT_BUMPER_LEFT, &cm_handle_bumper_left);
	event_manager_register_handler(EVT_BUMPER_RIGHT, &cm_handle_bumper_right);

#define event_manager_register_and_enable_x(name, y, enabled) \
	event_manager_register_handler(y, &cm_handle_ ##name); \
	event_manager_enable_handler(y, enabled);

	/* OBS */
	event_manager_register_and_enable_x(obs_front, EVT_OBS_FRONT, true);
	//event_manager_register_and_enable_x(obs_left, EVT_OBS_LEFT, true);
	//event_manager_register_and_enable_x(obs_right, EVT_OBS_RIGHT, true);

	/* Cliff */
	event_manager_register_and_enable_x(cliff_all, EVT_CLIFF_ALL, true);
	event_manager_register_and_enable_x(cliff_front_left, EVT_CLIFF_FRONT_LEFT, true);
	event_manager_register_and_enable_x(cliff_front_right, EVT_CLIFF_FRONT_RIGHT, true);
	event_manager_register_and_enable_x(cliff_left_right, EVT_CLIFF_LEFT_RIGHT, true);
	event_manager_register_and_enable_x(cliff_front, EVT_CLIFF_FRONT, true);
	event_manager_register_and_enable_x(cliff_left, EVT_CLIFF_LEFT, true);
	event_manager_register_and_enable_x(cliff_right, EVT_CLIFF_RIGHT, true);

	/* RCON */
	event_manager_register_and_enable_x(rcon, EVT_RCON, false);
/*
	event_manager_register_and_enable_x(rcon_front_left, EVT_RCON_FRONT_LEFT, false);
	event_manager_register_and_enable_x(rcon_front_left2, EVT_RCON_FRONT_LEFT2, false);
	event_manager_register_and_enable_x(rcon_front_right, EVT_RCON_FRONT_RIGHT, false);
	event_manager_register_and_enable_x(rcon_front_right2, EVT_RCON_FRONT_RIGHT2, false);
	event_manager_register_and_enable_x(rcon_left, EVT_RCON_LEFT, false);
	event_manager_register_and_enable_x(rcon_right, EVT_RCON_RIGHT, false);
*/

	/* Over Current */
	event_manager_enable_handler(EVT_OVER_CURRENT_BRUSH_LEFT, true);
	event_manager_register_and_enable_x(over_current_brush_main, EVT_OVER_CURRENT_BRUSH_MAIN, true);
	event_manager_enable_handler(EVT_OVER_CURRENT_BRUSH_RIGHT, true);
	event_manager_register_and_enable_x(over_current_wheel_left, EVT_OVER_CURRENT_WHEEL_LEFT, true);
	event_manager_register_and_enable_x(over_current_wheel_right, EVT_OVER_CURRENT_WHEEL_RIGHT, true);
	event_manager_register_and_enable_x(over_current_suction, EVT_OVER_CURRENT_SUCTION, true);

	/* Key */
	event_manager_register_and_enable_x(key_clean, EVT_KEY_CLEAN, true);

	/* Remote */
	event_manager_register_and_enable_x(remote_clean, EVT_REMOTE_CLEAN, true);
	event_manager_register_and_enable_x(remote_home, EVT_REMOTE_HOME, true);
	event_manager_register_and_enable_x(remote_wallfollow,EVT_REMOTE_WALL_FOLLOW, true);
	event_manager_register_and_enable_x(remote_spot, EVT_REMOTE_SPOT, true);
	event_manager_register_and_enable_x(remote_max, EVT_REMOTE_MAX, true);
	event_manager_register_and_enable_x(remote_direction, EVT_REMOTE_DIRECTION_RIGHT, true);
	event_manager_register_and_enable_x(remote_direction, EVT_REMOTE_DIRECTION_LEFT, true);
	event_manager_register_and_enable_x(remote_direction, EVT_REMOTE_DIRECTION_FORWARD, true);

	// Just enable the default handler.
	event_manager_enable_handler(EVT_REMOTE_PLAN, true);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_FORWARD, true);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_LEFT, true);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_RIGHT, true);
	event_manager_enable_handler(EVT_REMOTE_WALL_FOLLOW, true);

	/* Battery */
	event_manager_register_and_enable_x(battery_home, EVT_BATTERY_HOME, true);
	event_manager_register_and_enable_x(battery_low, EVT_BATTERY_LOW, true);

	/* Charge Status */
	event_manager_register_and_enable_x(charge_detect, EVT_CHARGE_DETECT, true);
	/* robot stuck */
	event_manager_enable_handler(EVT_ROBOT_SLIP,true);
	/* lidar bumper */
	//event_manager_enable_handler(EVT_LIDAR_BUMPER,true);
	/* Slam Error */
	event_manager_enable_handler(EVT_SLAM_ERROR, true);

	event_manager_enable_handler(EVT_LASER_STUCK, true);
#undef event_manager_register_and_enable_x

	event_manager_set_enable(true);
}

void cm_unregister_events()
{
	ROS_INFO("%s %d: Unregister events", __FUNCTION__, __LINE__);
#define event_manager_register_and_disable_x(x) \
	event_manager_register_handler(x, NULL); \
	event_manager_enable_handler(x, false);

	/* Bumper */
	event_manager_register_and_disable_x(EVT_BUMPER_ALL);
	event_manager_register_and_disable_x(EVT_BUMPER_LEFT);
	event_manager_register_and_disable_x(EVT_BUMPER_RIGHT);

	/* OBS */
	event_manager_register_and_disable_x(EVT_OBS_FRONT);
	//event_manager_register_and_disable_x(EVT_OBS_LEFT);
	//event_manager_register_and_disable_x(EVT_OBS_RIGHT);

	/* Cliff */
	event_manager_register_and_disable_x(EVT_CLIFF_ALL);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT_LEFT);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT_RIGHT);
	event_manager_register_and_disable_x(EVT_CLIFF_LEFT_RIGHT);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT);
	event_manager_register_and_disable_x(EVT_CLIFF_LEFT);
	event_manager_register_and_disable_x(EVT_CLIFF_RIGHT);

	/* RCON */
	event_manager_register_and_disable_x(EVT_RCON);
/*
	event_manager_register_and_disable_x(EVT_RCON_FRONT_LEFT);
	event_manager_register_and_disable_x(EVT_RCON_FRONT_LEFT2);
	event_manager_register_and_disable_x(EVT_RCON_FRONT_RIGHT);
	event_manager_register_and_disable_x(EVT_RCON_FRONT_RIGHT2);
	event_manager_register_and_disable_x(EVT_RCON_LEFT);
	event_manager_register_and_disable_x(EVT_RCON_RIGHT);
*/

	/* Over Current */
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_BRUSH_LEFT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_BRUSH_MAIN);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_BRUSH_RIGHT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_WHEEL_LEFT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_WHEEL_RIGHT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_SUCTION);

	/* Key */
	event_manager_register_and_disable_x(EVT_KEY_CLEAN);

	/* Remote */
	event_manager_register_and_disable_x(EVT_REMOTE_PLAN);
	event_manager_register_and_disable_x(EVT_REMOTE_CLEAN);
	event_manager_register_and_disable_x(EVT_REMOTE_HOME);
	event_manager_register_and_disable_x(EVT_REMOTE_SPOT);
	event_manager_register_and_disable_x(EVT_REMOTE_MAX);
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_LEFT);
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_RIGHT);
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_FORWARD);

	/* Battery */
	event_manager_register_and_disable_x(EVT_BATTERY_HOME);
	event_manager_register_and_disable_x(EVT_BATTERY_LOW);

	/* Charge Status */
	event_manager_register_and_disable_x(EVT_CHARGE_DETECT);

	/* Slam Error */
	event_manager_register_and_disable_x(EVT_SLAM_ERROR);
	/* robot slip */
	event_manager_register_and_disable_x(EVT_ROBOT_SLIP);
	/* lidar bumper*/
	//event_manager_register_and_disable_x(EVT_LIDAR_BUMPER);
	// Laser stuck
	event_manager_register_and_disable_x(EVT_LASER_STUCK);

#undef event_manager_register_and_disable_x

	event_manager_set_enable(false);
}

void cm_set_event_manager_handler_state(bool state)
{
	event_manager_enable_handler(EVT_BUMPER_ALL, state);
	event_manager_enable_handler(EVT_BUMPER_LEFT, state);
	event_manager_enable_handler(EVT_BUMPER_RIGHT, state);

	event_manager_enable_handler(EVT_OBS_FRONT, state);
	//event_manager_enable_handler(EVT_OBS_LEFT, state);
	//event_manager_enable_handler(EVT_OBS_RIGHT, state);

	event_manager_enable_handler(EVT_RCON, state);
/*
	event_manager_enable_handler(EVT_RCON_FRONT_LEFT, state);
	event_manager_enable_handler(EVT_RCON_FRONT_LEFT2, state);
	event_manager_enable_handler(EVT_RCON_FRONT_RIGHT, state);
	event_manager_enable_handler(EVT_RCON_FRONT_RIGHT2, state);
	event_manager_enable_handler(EVT_RCON_LEFT, state);
	event_manager_enable_handler(EVT_RCON_RIGHT, state);
*/
}

void cm_event_manager_turn(bool state)
{
	event_manager_enable_handler(EVT_BUMPER_ALL, state);
	event_manager_enable_handler(EVT_BUMPER_LEFT, state);
	event_manager_enable_handler(EVT_BUMPER_RIGHT, state);
}

void cm_handle_bumper_all(bool state_now, bool state_last)
{
//	ev.bumper_triggered = BLOCK_ALL;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !ev.bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, get_bumper_status(), state_now ? "true" : "false", state_last ? "true" : "false");

}

void cm_handle_bumper_left(bool state_now, bool state_last)
{
//	if(ev.bumper_triggered != 0)
//		return;
//	ev.bumper_triggered = BLOCK_LEFT;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !ev.bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, get_bumper_status(), state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_bumper_right(bool state_now, bool state_last)
{
//	if(ev.bumper_triggered != 0)
//		return;
//
//	ev.bumper_triggered = BLOCK_RIGHT;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !ev.bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, get_bumper_status(), state_now ? "true" : "false", state_last ? "true" : "false");
}

/* OBS */
void cm_handle_obs_front(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
//	ev.obs_triggered = Status_Front_OBS;
}

void cm_handle_obs_left(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
//	ev.obs_triggered = Status_Left_OBS;
}

void cm_handle_obs_right(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
//	ev.obs_triggered = Status_Right_OBS;
}

/* Cliff */
void cm_handle_cliff_all(bool state_now, bool state_last)
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

void cm_handle_cliff_front_left(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_LF;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");

}

void cm_handle_cliff_front_right(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_RF;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");

}

void cm_handle_cliff_left_right(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_LR;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}
//
//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_cliff_front(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_Front;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}
//
//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_cliff_left(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_Left;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_cliff_right(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_Right;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

/* RCON */
void cm_handle_rcon(bool state_now, bool state_last)
{
/*
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (cs_is_going_home()) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}
	if(mt_is_follow_wall()){
		if (!(get_rcon_status() & (RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)))
			return;
	}
	else if (mt_is_linear())
		// Since we have front left 2 and front right 2 rcon receiver, seems it is not necessary to handle left or right rcon receives home signal.
		if (!(get_rcon_status() & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)))
		return;

	ev.rcon_triggered = get_rcon_trig_();
	if(ev.rcon_triggered != 0){
		map_set_rcon();
	}
	reset_rcon_status();*/
}

/* Over Current */
//void cm_handle_over_current_brush_left(bool state_now, bool state_last)
//{
//	static uint8_t stop_cnt = 0;
//
//	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
//	if(!robot::instance()->getLbrushOc()) {
//		g_oc_brush_left_cnt = 0;
//		if (stop_cnt++ > 250) {
//			set_left_brush_pwm(30);
//		}
//		return;
//	}
//
//	stop_cnt = 0;
//	if (g_oc_brush_left_cnt++ > 40) {
//		g_oc_brush_left_cnt = 0;
//		set_left_brush_pwm(0);
//		ROS_WARN("%s %d: left brush over current", __FUNCTION__, __LINE__);
//	}
//}

void cm_handle_over_current_brush_main(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!robot::instance()->getMbrushOc()){
		g_oc_brush_main_cnt = 0;
		return;
	}

	if (g_oc_brush_main_cnt++ > 40) {
		g_oc_brush_main_cnt = 0;
		ROS_WARN("%s %d: main brush over current", __FUNCTION__, __LINE__);

		if (self_check(Check_Main_Brush) == 1) {
			ev.oc_brush_main = true;
			ev.fatal_quit = true;
		}
    }
}

//void cm_handle_over_current_brush_right(bool state_now, bool state_last)
//{
//	static uint8_t stop_cnt = 0;
//
//	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
//
//	if(!robot::instance()->getRbrushOc()) {
//		g_oc_brush_right_cnt = 0;
//		if (stop_cnt++ > 250) {
//			set_right_brush_pwm(30);
//		}
//		return;
//	}
//
//	stop_cnt = 0;
//	if (g_oc_brush_right_cnt++ > 40) {
//		g_oc_brush_right_cnt = 0;
//		set_right_brush_pwm(0);
//		ROS_WARN("%s %d: reft brush over current", __FUNCTION__, __LINE__);
//	}
//}

void cm_handle_over_current_wheel_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) robot::instance()->getLwheelCurrent() < Wheel_Stall_Limit) {
        g_oc_wheel_left_cnt = 0;
		return;
	}

	if (g_oc_wheel_left_cnt++ > 40){
		g_oc_wheel_left_cnt = 0;
		ROS_WARN("%s %d: left wheel over current, \033[1m%u mA\033[0m", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getLwheelCurrent());

		ev.oc_wheel_left = true;
	}
}

void cm_handle_over_current_wheel_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) robot::instance()->getRwheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_right_cnt = 0;
		return;
	}

	if (g_oc_wheel_right_cnt++ > 40){
		g_oc_wheel_right_cnt = 0;
		ROS_WARN("%s %d: right wheel over current, \033[1m%u mA\033[0m", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getRwheelCurrent());

		ev.oc_wheel_right = true;
	}
}

void cm_handle_over_current_suction(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!robot::instance()->getVacuumOc()) {
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
void cm_handle_key_clean(bool state_now, bool state_last)
{
	time_t start_time;
	bool reset_manual_pause = false;

	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (ev.slam_error)
	{
		beep_for_command(INVALID);
		while (get_key_press() & KEY_CLEAN)
		{
			usleep(20000);
		}
		ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
		reset_touch();
		return;
	}

	beep_for_command(VALID);
	set_wheel_speed(0, 0);
	ev.key_clean_pressed = true;

	if(cm_is_navigation())
		robot::instance()->setManualPause();

	start_time = time(NULL);
	while (get_key_press() & KEY_CLEAN)
	{
		if (cm_is_navigation() && time(NULL) - start_time > 3) {
			if (!reset_manual_pause)
			{
				beep_for_command(VALID);
				reset_manual_pause = true;
				robot::instance()->resetManualPause();
				ROS_WARN("%s %d: Manual pause has been reset.", __FUNCTION__, __LINE__);
			}
		}
		else
			ROS_DEBUG("%s %d: Key clean is not released.", __FUNCTION__, __LINE__);
		usleep(20000);
	}

	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
	reset_touch();
}

/* Remote */

void cm_handle_remote_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (ev.slam_error)
	{
		beep_for_command(INVALID);
		reset_rcon_remote();
		return;
	}
	beep_for_command(VALID);
	ev.key_clean_pressed = true;
	if(cm_is_navigation()){
		robot::instance()->setManualPause();
	}
	reset_rcon_remote();
}

void cm_handle_remote_home(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_motion_init_succeeded && !cs_is_going_home() && !cm_should_self_check() && !ev.slam_error && !robot::instance()->isManualPaused()) {

		if( SpotMovement::instance()->getSpotType()  == NORMAL_SPOT){
			beep_for_command(INVALID);
		}
		else{
			ev.remote_home = true;
			beep_for_command(VALID);
			if (SpotMovement::instance()->getSpotType() == CLEAN_SPOT){
				SpotMovement::instance()->spotDeinit();
			}
		}
		ROS_INFO("ev.remote_home = %d", ev.remote_home);
	}
	else {
		beep_for_command(INVALID);
		ROS_INFO("ev.remote_home = %d", ev.remote_home);
	}
	reset_rcon_remote();
}

void cm_handle_remote_spot(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!g_motion_init_succeeded || !cm_is_navigation()
		|| cs_is_going_home() || cm_should_self_check() || ev.slam_error || robot::instance()->isManualPaused()
		|| time(NULL) - last_time_remote_spot < 3)
		beep_for_command(INVALID);
	else
	{
		ev.remote_spot = true;
		last_time_remote_spot = time(NULL);
		beep_for_command(VALID);
	}

	reset_rcon_remote();
}

void cm_handle_remote_wallfollow(bool state_now,bool state_last)
{
	ROS_WARN("%s,%d: is called.",__FUNCTION__,__LINE__);
	beep_for_command(INVALID);
	reset_rcon_remote();
}

void cm_handle_remote_max(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_motion_init_succeeded && !cs_is_going_home() && !cm_should_self_check() && SpotMovement::instance()->getSpotType() == NO_SPOT && !ev.slam_error && !robot::instance()->isManualPaused())
	{
		beep_for_command(VALID);
		switch_vac_mode(true);
	}
	else
		beep_for_command(INVALID);
	reset_rcon_remote();
}

void cm_handle_remote_direction(bool state_now,bool state_last)
{
	ROS_WARN("%s,%d: is called.",__FUNCTION__,__LINE__);
	// For Debug
	// ev.battrey_home = true;
	// path_set_continue_cell(map_get_curr_cell());
	// robot::instance()->setLowBatPause();
	beep_for_command(INVALID);
	reset_rcon_remote();
}

/* Battery */
void cm_handle_battery_home(bool state_now, bool state_last)
{
	if (g_motion_init_succeeded && !cs_is_going_home()) {
		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m", __FUNCTION__, __LINE__,
						 robot::instance()->getBatteryVoltage());
		ev.battrey_home = true;

		if (get_vac_mode() == Vac_Max) {
			switch_vac_mode(false);
		}
#if CONTINUE_CLEANING_AFTER_CHARGE
		if (SpotMovement::instance()->getSpotType() != NORMAL_SPOT ){
			path_set_continue_cell(map_get_curr_cell());
			robot::instance()->setLowBatPause();
		}
#endif 
    }
}

void cm_handle_battery_low(bool state_now, bool state_last)
{
    uint8_t         v_pwr, s_pwr, m_pwr;
    uint16_t        t_vol;

    ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_battery_low_cnt++ > 50) {
		t_vol = get_battery_voltage();
		ROS_WARN("%s %d: low battery, battery < %umv is detected.", __FUNCTION__, __LINE__,t_vol);

		if (cs_is_going_home()) {
			v_pwr = Home_Vac_Power / t_vol;
			s_pwr = Home_SideBrush_Power / t_vol;
			m_pwr = Home_MainBrush_Power / t_vol;
		} else {
			v_pwr = Clean_Vac_Power / t_vol;
			s_pwr = Clean_SideBrush_Power / t_vol;
			m_pwr = Clean_MainBrush_Power / t_vol;
		}

		g_battery_low_cnt = 0;
		set_bldc_speed(v_pwr);
		set_side_brush_pwm(s_pwr, s_pwr);
		set_main_brush_pwm(m_pwr);

		ev.fatal_quit = true;
		ev.battery_low = true;
	}
}

void cm_handle_charge_detect(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: Detect charger: %d, g_charge_detect_cnt: %d.", __FUNCTION__, __LINE__, robot::instance()->getChargeStatus(), g_charge_detect_cnt);
	if (((cm_is_exploration() || cm_is_go_charger() || cs_is_going_home()) && robot::instance()->getChargeStatus()) ||
		(!cm_is_exploration() && robot::instance()->getChargeStatus() == 3))
	{
		if (g_charge_detect_cnt++ > 2)
		{
			ev.charge_detect = robot::instance()->getChargeStatus();
			if (!cm_is_exploration() && robot::instance()->getChargeStatus() == 3)
				ev.fatal_quit = true;
			ROS_WARN("%s %d: ev.charge_detect has been set to %d.", __FUNCTION__, __LINE__, ev.charge_detect);
			g_charge_detect_cnt = 0;
		}
	}
}
