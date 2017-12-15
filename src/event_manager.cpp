#include "pp.h"
int g_bumper_cnt = 0;
/* OBS */
uint8_t g_cliff_all_cnt = 0;
int g_cliff_cnt = 0;
/* RCON */
//int ev.rcon_triggered = 0;
/* Over Current */
uint8_t g_oc_wheel_left_cnt = 0;
uint8_t g_oc_wheel_right_cnt = 0;
uint8_t g_oc_suction_cnt = 0;

uint8_t g_battery_low_cnt = 0;
uint8_t g_charge_detect_cnt = 0;
bool g_plan_activated = false;


/* robot slip & stuck */
uint8_t g_slip_cnt = 0;
bool g_robot_slip = false;
bool g_robot_slip_enable = false;
bool g_robot_stuck = false;

Ev_t ev;
/* lidar bumper */
//bool g_lidar_bumper = false;
//bool g_lidar_bumper_jam =false;
//int g_lidar_bumper_cnt = 0;

// lidar stuck


static EventActionType	eat;

pthread_mutex_t	new_event_mtx;
pthread_cond_t new_event_cond = PTHREAD_COND_INITIALIZER;

bool g_event_manager_enabled = false;
bool g_event_handler_status = false;
pthread_mutex_t	event_handler_mtx;
pthread_cond_t event_handler_cond = PTHREAD_COND_INITIALIZER;

static bool g_new_event_status[EVT_MAX];

PEHF p_handler[EVT_MAX];
EventHandle deh;
void event_manager_init()
{
	int	j;

	g_event_manager_enabled = g_event_handler_status = false;

	event_manager_reset_status();

		for (j = 0; j < EVT_MAX; j++) {
//			eat.handler[j] = NULL;
//			p_handler[j] = NULL;
//			eat.handler_enabled[j] = false;
			g_new_event_status[j] = false;
		}
	p_handler[EVT_BUMPER_ALL] = &EventHandle::bumperAll;
	p_handler[EVT_BUMPER_LEFT] = &EventHandle::bumperLeft;
	p_handler[EVT_BUMPER_RIGHT] = &EventHandle::bumperRight;

	p_handler[EVT_OBS_FRONT] = &EventHandle::obsFront;
	p_handler[EVT_OBS_LEFT] = &EventHandle::obsLeft;
	p_handler[EVT_OBS_RIGHT] = &EventHandle::obsRight;
	p_handler[EVT_OBS_WALL_LFET] = &EventHandle::obsWallLeft;
	p_handler[EVT_OBS_WALL_RIGHT] = &EventHandle::obsWallRight;

	p_handler[EVT_CLIFF_ALL] = &EventHandle::cliffAll;
	p_handler[EVT_CLIFF_FRONT_LEFT] = &EventHandle::cliffFrontLeft;
	p_handler[EVT_CLIFF_FRONT_RIGHT] = &EventHandle::cliffFrontRight;
	p_handler[EVT_CLIFF_LEFT_RIGHT] = &EventHandle::cliffLeftRight;
	p_handler[EVT_CLIFF_LEFT] = &EventHandle::cliffLeft;
	p_handler[EVT_CLIFF_RIGHT] = &EventHandle::cliffRight;
	p_handler[EVT_CLIFF_FRONT] = &EventHandle::cliffFront;

	p_handler[EVT_RCON] = &EventHandle::rcon;

	p_handler[EVT_OVER_CURRENT_BRUSH_LEFT] = &EventHandle::overCurrentBrushLeft;
	p_handler[EVT_OVER_CURRENT_BRUSH_MAIN] = &EventHandle::overCurrentBrushMain;
	p_handler[EVT_OVER_CURRENT_BRUSH_RIGHT] = &EventHandle::overCurrentBrushRight;
	p_handler[EVT_OVER_CURRENT_WHEEL_LEFT] = &EventHandle::overCurrentWheelLeft;
	p_handler[EVT_OVER_CURRENT_WHEEL_RIGHT] = &EventHandle::overCurrentWheelRight;
	p_handler[EVT_OVER_CURRENT_SUCTION] = &EventHandle::overCurrentSuction;

	p_handler[EVT_KEY_CLEAN] = &EventHandle::keyClean;

	p_handler[EVT_REMOTE_PLAN] = &EventHandle::remotePlan;
	p_handler[EVT_REMOTE_CLEAN] = &EventHandle::remoteClean;
	p_handler[EVT_REMOTE_HOME] = &EventHandle::remoteHome;

//	handler[EVT_REMOTE_DIRECTION_BACKWARD]=handler_remote_direction_forward;
	p_handler[EVT_REMOTE_DIRECTION_FORWARD] = &EventHandle::remoteDirectionForward;
	p_handler[EVT_REMOTE_DIRECTION_LEFT] = &EventHandle::remoteDirectionLeft;
	p_handler[EVT_REMOTE_DIRECTION_RIGHT] = &EventHandle::remoteDirectionRight;

	p_handler[EVT_REMOTE_WALL_FOLLOW] = &EventHandle::remoteWallFollow;
	p_handler[EVT_REMOTE_SPOT] = &EventHandle::remoteSpot;

	p_handler[EVT_REMOTE_MAX] = &EventHandle::remoteMax;
//	handler[EVT_REMOTE_TIMER]=handler_remote_max;
//	handler[EVT_WATER_TANK]=handler_remote_max;

	p_handler[EVT_BATTERY_HOME] = &EventHandle::batteryHome;
	p_handler[EVT_BATTERY_LOW] = &EventHandle::batteryLow;

	p_handler[EVT_CHARGE_DETECT] = &EventHandle::chargeDetect;

	p_handler[EVT_ROBOT_SLIP] = &EventHandle::robotSlip;

//	handler[EVT_LIDAR_BUMPER]=handler_lidar_stuck;
	p_handler[EVT_LIDAR_STUCK] = &EventHandle::lidarStuck;
	eat.peh = &deh;
}

void event_manager_set_enable(bool enable)
{
	g_event_manager_enabled = enable;
	if (!enable)
	{
		//ROS_WARN("%s %d: Disable all event under manager mode_:%d", __FUNCTION__, __LINE__, evt_mgr_mode);
		for (int i = 0; i < EVT_MAX; i++) {
//			eat.handler[i] = NULL;
//			eat.handler_enabled[i] = false;
			g_new_event_status[i] = false;
		}
	}
}


void *event_manager_thread(void *data)
{
	bool set = false;
	bool status[EVT_MAX];
	auto evt_set_status_x = [&](EventType x)
	{
		status[x] = true;
		set = true;
	};

	pthread_detach(pthread_self());

	while (ros::ok()) {
		if (g_event_manager_enabled == false) {
			usleep(10000);
			continue;
		}

		set = false;
		for (int i = 0; i < EVT_MAX; i++) {
			status[i] = false;
		}

		pthread_mutex_lock(&serial_data_ready_mtx);
		pthread_cond_wait(&serial_data_ready_cond, &serial_data_ready_mtx);
		pthread_mutex_unlock(&serial_data_ready_mtx);

		//ROS_DEBUG("%s %d: wake up by serial data arrive", __FUNCTION__, __LINE__);


		/* Bumper */
		if (bumper.get_status() == BLOCK_ALL) {
			ROS_DEBUG("%s %d: setting event:all bumper trig ", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BUMPER_ALL);
		} else if (bumper.get_status() & BLOCK_LEFT) {
			ROS_DEBUG("%s %d: setting event: left bumper trig", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BUMPER_LEFT);
		} else if (bumper.get_status() & BLOCK_RIGHT) {
			ROS_DEBUG("%s %d: setting event: right bumper trig", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BUMPER_RIGHT);
		}

		/* OBS */
		if (obs.getFront() > obs.getFrontTrigValue() + 1700) {
			ROS_DEBUG("%s %d: setting event: front obs", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OBS_FRONT);
		}
		if (obs.getLeft() > obs.getLeftTrigValue() + 200) {
			ROS_DEBUG("%s %d: setting event: left obs", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OBS_LEFT);
		}
		if (obs.getRight() > obs.getRightTrigValue() + 200) {
			ROS_DEBUG("%s %d: setting event: right obs", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OBS_RIGHT);
		}

		/* Cliff */
		if (cliff.get_status() == BLOCK_ALL) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_ALL);
		} else if (cliff.get_status() == (BLOCK_FRONT | BLOCK_LEFT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_FRONT_LEFT);
		} else if (cliff.get_status() == (BLOCK_FRONT | BLOCK_RIGHT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_FRONT_RIGHT);
		} else if (cliff.get_status() == (BLOCK_LEFT | BLOCK_RIGHT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_LEFT_RIGHT);
		} else if (cliff.get_status() == (BLOCK_FRONT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_FRONT);
		} else if (cliff.get_status() == (BLOCK_LEFT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_LEFT);
		} else if (cliff.get_status() == (BLOCK_RIGHT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CLIFF_RIGHT);
		}

		/* RCON */
		if (c_rcon.getStatus()) {
			//ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON);
		}
/*
		if ((Get_Rcon_Status() & RconFL_HomeT) == RconFL_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_FRONT_LEFT)
		} else if ((Get_Rcon_Status() & RconFL2_HomeT) == RconFL2_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_FRONT_LEFT2)
		} else if ((Get_Rcon_Status() & RconFR_HomeT) == RconFR_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_FRONT_RIGHT)
		} else if ((Get_Rcon_Status() & RconFR2_HomeT) == RconFR2_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_FRONT_RIGHT2)
		} else if ((Get_Rcon_Status() & RconL_HomeT) == RconL_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_LEFT)
		} else if ((c_rcon.getStatus() & RconR_HomeT) == RconR_HomeT) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_RCON_RIGHT)
		}
*/

		/* Over Current */
		if (1/* || robot::instance()->getLbrushOc()*/) {
			//ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_BRUSH_LEFT);
		}
		if (brush.getMainOc()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_BRUSH_MAIN);
		}
		if (1/* || robot::instance()->getRbrushOc()*/) {
			//ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_BRUSH_RIGHT);
		}
		if ((uint32_t) wheel.getLeftWheelCurrent() > Wheel_Stall_Limit) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_WHEEL_LEFT);
		}
		if ((uint32_t) wheel.getRightWheelCurrent() > Wheel_Stall_Limit) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_WHEEL_RIGHT);
		}
		if (vacuum.getOc()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_OVER_CURRENT_SUCTION);
		}

		/* Key */
		if (key.getTriggerStatus()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_KEY_CLEAN);
		}

		if (robot_timer.getPlanStatus()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_PLAN);
		}

		/* Remote */
		if (remote.isKeyTrigger(REMOTE_CLEAN)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_CLEAN);
		}
		if (remote.isKeyTrigger(REMOTE_HOME)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_HOME);
		}
		if (remote.isKeyTrigger(REMOTE_FORWARD)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_DIRECTION_FORWARD);
		}
		if (remote.isKeyTrigger(REMOTE_LEFT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_DIRECTION_LEFT);
		}
		if (remote.isKeyTrigger(REMOTE_RIGHT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_DIRECTION_RIGHT);
		}
		if (remote.isKeyTrigger(REMOTE_SPOT)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_SPOT);
		}
		if (remote.isKeyTrigger(REMOTE_MAX)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_MAX);
		}
		if (remote.isKeyTrigger(REMOTE_WALL_FOLLOW)) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_REMOTE_WALL_FOLLOW);
		}

		/* Battery */
		if (battery.getVoltage() && battery.getVoltage() < LOW_BATTERY_GO_HOME_VOLTAGE) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BATTERY_HOME);
		}
		if (battery.getVoltage() < LOW_BATTERY_STOP_VOLTAGE) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_BATTERY_LOW);
		}

		/* Charge Status */
		if (charger.getChargeStatus()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_CHARGE_DETECT);
		}

		/* robot slip */
		if(lidar_is_robot_slip()){
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_ROBOT_SLIP);
		}
		/*
		if(robot::instance()->getLidarBumper()){
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_LIDAR_BUMPER)
		}
		*/

		// Lidar stuck
		if (lidar_is_stuck()) {
			ROS_DEBUG("%s %d: setting event:", __FUNCTION__, __LINE__);
			evt_set_status_x(EVT_LIDAR_STUCK);
		}

		if (set) {
			//ROS_INFO("%s %d: going to broadcase new event", __FUNCTION__, __LINE__);
			pthread_mutex_lock(&new_event_mtx);

			memcpy(g_new_event_status, status, sizeof(bool) * EVT_MAX);
			pthread_cond_broadcast(&new_event_cond);

			pthread_mutex_unlock(&new_event_mtx);
		}
	}

	ROS_ERROR("%s %d: exit\n", __FUNCTION__, __LINE__);
}

void *event_handler_thread(void *data) {
	bool status_now[EVT_MAX], status_last[EVT_MAX];

	auto evt_handle_event_x = [&](EventType x)
	{
//		if (eat.handler_enabled[x]) {
//			if (eat.handler[x] == NULL) {
				(eat.peh->*p_handler[x])(status_now[x], status_last[x]);
//			} else {
//				eat.handler[x](status_now[x], status_last[x]);
//			}
//		}
	};

	auto evt_handle_check_event = [&](EventType x)
	{
		if (status_now[x]) {
			evt_handle_event_x(x);
		}
	};

	pthread_detach(pthread_self());

	while (ros::ok()) {
		pthread_mutex_lock(&new_event_mtx);
		pthread_cond_wait(&new_event_cond, &new_event_mtx);

		memcpy(status_last, status_now, sizeof(bool) * EVT_MAX);
		memcpy(status_now, g_new_event_status, sizeof(bool) * EVT_MAX);
		for (int i = 0; i < EVT_MAX; i++) {
			g_new_event_status[i] = false;
		}

		pthread_mutex_unlock(&new_event_mtx);

		pthread_mutex_lock(&event_handler_mtx);
		g_event_handler_status = true;
		pthread_cond_broadcast(&event_handler_cond);
		pthread_mutex_unlock(&event_handler_mtx);

		//ROS_DEBUG("%s %d: handler thread is up, new event to handle", __FUNCTION__, __LINE__);

		/* Bumper */
		if (status_now[EVT_BUMPER_ALL]) {
			evt_handle_event_x(EVT_BUMPER_ALL);
		} else if (status_now[EVT_BUMPER_RIGHT]) {
			evt_handle_event_x(EVT_BUMPER_RIGHT);
		} else if (status_now[EVT_BUMPER_LEFT]) {
			evt_handle_event_x(EVT_BUMPER_LEFT);
		}

		/* OBS */
		evt_handle_check_event(EVT_OBS_FRONT);
		evt_handle_check_event(EVT_OBS_LEFT);
		evt_handle_check_event(EVT_OBS_RIGHT);

		/* Cliff */
		if (status_now[EVT_CLIFF_ALL]) {
            evt_handle_event_x(EVT_CLIFF_ALL);
		} else if (status_now[EVT_CLIFF_FRONT_LEFT]) {
            evt_handle_event_x(EVT_CLIFF_FRONT_LEFT);
		} else if (status_now[EVT_CLIFF_FRONT_RIGHT]) {
            evt_handle_event_x(EVT_CLIFF_FRONT_RIGHT);
		} else if (status_now[EVT_CLIFF_LEFT_RIGHT]) {
            evt_handle_event_x(EVT_CLIFF_LEFT_RIGHT);
		} else if (status_now[EVT_CLIFF_FRONT]) {
            evt_handle_event_x(EVT_CLIFF_FRONT);
		} else if (status_now[EVT_CLIFF_LEFT]) {
            evt_handle_event_x(EVT_CLIFF_LEFT);
		} else if (status_now[EVT_CLIFF_RIGHT]) {
            evt_handle_event_x( EVT_CLIFF_RIGHT);
		}

		/* RCON */
		evt_handle_check_event(EVT_RCON);
/*
		if (status_now[EVT_RCON_FRONT_LEFT] == true) {
            evt_handle_event_x(rcon_front_left, EVT_RCON_FRONT_LEFT)
		} else if (status_now[EVT_RCON_FRONT_LEFT2] == true) {
            evt_handle_event_x(rcon_front_left2, EVT_RCON_FRONT_LEFT2)
		} else if (status_now[EVT_RCON_FRONT_RIGHT] == true) {
            evt_handle_event_x(rcon_front_right, EVT_RCON_FRONT_RIGHT)
		} else if (status_now[EVT_RCON_FRONT_RIGHT2] == true) {
            evt_handle_event_x(rcon_front_right2, EVT_RCON_FRONT_RIGHT2)
		} else if (status_now[EVT_RCON_LEFT] == true) {
            evt_handle_event_x(rcon_left, EVT_RCON_LEFT)
		} else if (status_now[EVT_RCON_RIGHT] == true) {
            evt_handle_event_x(rcon_right, EVT_RCON_RIGHT)
		}
*/

		/* Over Current */
		evt_handle_check_event(EVT_OVER_CURRENT_BRUSH_LEFT);
		evt_handle_check_event(EVT_OVER_CURRENT_BRUSH_MAIN);
		evt_handle_check_event(EVT_OVER_CURRENT_BRUSH_RIGHT);
		evt_handle_check_event(EVT_OVER_CURRENT_WHEEL_LEFT);
		evt_handle_check_event(EVT_OVER_CURRENT_WHEEL_RIGHT);
		evt_handle_check_event(EVT_OVER_CURRENT_SUCTION);
		
		/* Key */
		evt_handle_check_event(EVT_KEY_CLEAN);

		/* Remote */
		evt_handle_check_event(EVT_REMOTE_PLAN);
		evt_handle_check_event(EVT_REMOTE_CLEAN);
		evt_handle_check_event(EVT_REMOTE_HOME);
		evt_handle_check_event(EVT_REMOTE_DIRECTION_FORWARD);
		evt_handle_check_event(EVT_REMOTE_DIRECTION_LEFT);
		evt_handle_check_event(EVT_REMOTE_DIRECTION_RIGHT);
		evt_handle_check_event(EVT_REMOTE_SPOT);
		evt_handle_check_event(EVT_REMOTE_MAX);
		evt_handle_check_event(EVT_REMOTE_WALL_FOLLOW);

		/* Battery */
		evt_handle_check_event(EVT_BATTERY_HOME);
		evt_handle_check_event(EVT_BATTERY_LOW);

		/* Charge Status */
		evt_handle_check_event(EVT_CHARGE_DETECT);

		/* robot slip*/
		evt_handle_check_event(EVT_ROBOT_SLIP);

		/* lidar bumper*/
		//evt_handle_check_event(EVT_LIDAR_BUMPER,lidar_bumper)

		// Lidar stuck
		evt_handle_check_event(EVT_LIDAR_STUCK);

		pthread_mutex_lock(&event_handler_mtx);
		g_event_handler_status = false;
		pthread_cond_broadcast(&event_handler_cond);
		pthread_mutex_unlock(&event_handler_mtx);
	}
	ROS_ERROR("%s %d: exit\n", __FUNCTION__, __LINE__);
}

//void event_manager_set_current_mode(EventModeType mode_)
//{
//	evt_mgr_mode = mode_;
//}

void event_manager_register_handler(EventHandle* eh)
{
	eat.peh = eh;
}

uint8_t event_manager_check_event(bool *p_eh_status_now, bool *p_eh_status_last)
{
	struct timespec	ts;

	clock_gettime(CLOCK_REALTIME, &ts);
	timespec_add_ns(ts, _10MS);

	pthread_mutex_lock(&event_handler_mtx);
	pthread_cond_timedwait(&event_handler_cond, &event_handler_mtx, &ts);

	*p_eh_status_last = *p_eh_status_now;
	*p_eh_status_now = g_event_handler_status;
	pthread_mutex_unlock(&event_handler_mtx);

	if (*p_eh_status_now == true) {
		if (*p_eh_status_last == false) {
//			ROS_WARN("%s %d: event manager handling event, process pause!\n", __FUNCTION__, __LINE__);
		}
		return 1;
	} else
	if (*p_eh_status_last == true) {
//		ROS_WARN("%s %d: event manager finished handling event, process resume!\n", __FUNCTION__, __LINE__);
	}

	return 0;
}

void event_manager_reset_status(void)
{
	ev.fatal_quit = false;
	/* Bumper */
	ev.bumper_triggered = false;
	ev.bumper_jam = false;
	g_bumper_cnt = 0;
	/* OBS */
//	ev.obs_triggered = false;
	/* Cliff */
	ev.cliff_all_triggered = false;
	ev.cliff_triggered = 0;
	ev.cliff_jam = false;
	g_cliff_all_cnt = 0;
	g_cliff_cnt = 0;
	/* RCON */
	/* Over Current */
	ev.oc_brush_main = false;
	ev.oc_wheel_left = false;
	ev.oc_wheel_right = false;
	ev.oc_suction = false;
	brush.oc_left_cnt_ = 0;
	brush.oc_main_cnt_ = 0;
	brush.oc_right_cnt_ = 0;
	g_oc_wheel_left_cnt = 0;
	g_oc_wheel_right_cnt = 0;
	g_oc_suction_cnt = 0;
	/* Key */
	ev.key_clean_pressed = false;
	ev.key_long_pressed = false;
	/* Remote */
	ev.remote_home = false;
	ev.remote_spot = false;
	ev.remote_wallfollow = false;
	ev.remote_direction_forward = false;
	ev.remote_direction_back = false;
	ev.remote_direction_left = false;
	ev.remote_direction_right = false;
	/* Battery */
	ev.battery_home = false;
	ev.battery_low = false;
	g_battery_low_cnt = 0;
	/* Charge Status */
	ev.charge_detect = 0;
	g_charge_detect_cnt = 0;
	/* Slam Error */
	ev.slam_error = false;
	/* robot stuck */
	//g_robot_stuck = false;
	g_robot_slip = false;
	g_slip_cnt = 0;
	/* tilt switch*/
	gyro.TiltCheckingEnable(false);
	ev.tilt_triggered = false;
	/* lidar bumper */
	//g_lidar_bumper = false;
	//g_lidar_bumper_cnt =0;
	//g_lidar_bumper_jam = false;
	// lidar stuck
	ev.lidar_stuck = false;
}

/* Below are the internal functions. */

/* Bumper */
void EventHandle::bumperAll(bool state_now, bool state_last)
{
/*	if (state_now == true && state_last == true) {
		bumper_all_cnt++;

		if (bumper_all_cnt > 2) {
			ROS_WARN("%s %d: all bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, bumper_all_cnt);
		}
	} else {
		bumper_all_cnt = 0;
	}

	move_back();
	wheel_stop();

	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, bumper.get_status());*/
}

void EventHandle::bumperLeft(bool state_now, bool state_last)
{
/*	if (state_now == true && state_last == true) {
		bumper_left_cnt++;

		if (bumper_left_cnt > 2) {
			ROS_WARN("%s %d: left bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, bumper_left_cnt);
		}
	} else {
		bumper_left_cnt = 0;
	}

	move_back();
	wheel_stop();
	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, bumper.get_status());*/
}

void EventHandle::bumperRight(bool state_now, bool state_last)
{
/*
	if (state_now == true && state_last == true) {
		bumper_right_cnt++;

		if (bumper_right_cnt > 2) {
			ROS_WARN("%s %d: right bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, bumper_right_cnt);
		}
	} else {
		bumper_right_cnt = 0;
	}

	move_back();
	wheel_stop();
	ROS_DEBUG("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, bumper.get_status());*/
}

/* OBS */
void EventHandle::obsFront(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//wheel_stop();
}

void EventHandle::obsLeft(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//wheel_stop();
}

void EventHandle::obsRight(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//wheel_stop();
}

void EventHandle::obsWallLeft(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//wheel_stop();
}

void EventHandle::obsWallRight(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	//move_back();
	//wheel_stop();
}

/* Cliff */
void EventHandle::cliffAll(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::cliffFrontLeft(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::cliffFrontRight(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::cliffLeftRight(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::cliffFront(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::cliffLeft(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::cliffRight(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* RCON */
void EventHandle::rcon(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
//	c_rcon.resetStatus();
}
void df_rcon(bool state_now, bool state_last)
{
	c_rcon.resetStatus();
}
/*
void EventHandle::rcon_front_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::rcon_front_left2(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::rcon_front_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::rcon_front_right2(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::rcon_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::rcon_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}
*/

/* Over Current */
void EventHandle::overCurrentBrushLeft(bool state_now, bool state_last)
{

}
void df_over_current_brush_left(bool state_now, bool state_last)
{
	//ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	if (!ev.fatal_quit && brush.leftIsStall())
	{
		error.set(ERROR_CODE_LEFTBRUSH);
		ev.fatal_quit = true;
		ROS_WARN("%s %d: Left brush stall, please check.", __FUNCTION__, __LINE__);
	}
}


void EventHandle::overCurrentBrushMain(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::overCurrentBrushRight(bool state_now, bool state_last)
{}
void df_over_current_brush_right(bool state_now, bool state_last)
{
	//ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	if (!ev.fatal_quit && brush.rightIsStall())
	{
		error.set(ERROR_CODE_RIGHTBRUSH);
		ev.fatal_quit = true;
		ROS_WARN("%s %d: Right brush stall, please check.", __FUNCTION__, __LINE__);
	}
}

void EventHandle::overCurrentWheelLeft(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::overCurrentWheelRight(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::overCurrentSuction(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* Key */
void EventHandle::keyClean(bool state_now, bool state_last)
{
//	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

/* Remote */
void EventHandle::remotePlan(bool state_now, bool state_last)
{

}
void df_remote_plan(bool state_now, bool state_last)
{
	if (robot_timer.getPlanStatus() == 1 || robot_timer.getPlanStatus() == 2)
	{
		ROS_WARN("%s %d: Remote plan is pressed.", __FUNCTION__, __LINE__);
		beeper.play_for_command(INVALID);
	}
	else if (robot_timer.getPlanStatus() == 3)
		ROS_WARN("%s %d: Plan is activated.", __FUNCTION__, __LINE__);

	robot_timer.resetPlanStatus();
	remote.reset();
}

void EventHandle::remoteClean(bool state_now, bool state_last)
{}
void df_remote_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote clean is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remoteHome(bool state_now, bool state_last)
{}
void df_remote_home(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote home is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remoteDirectionForward(bool state_now, bool state_last)
{}
void df_remote_direction_forward(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote forward is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remoteWallFollow(bool state_now, bool state_last)
{}
void df_remote_wall_follow(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote wall_follow is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remoteDirectionLeft(bool state_now, bool state_last)
{}
void df_remote_direction_left(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote left is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remoteDirectionRight(bool state_now, bool state_last)
{}
void df_remote_direction_right(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote right is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remoteSpot(bool state_now, bool state_last)
{}
void df_remote_spot(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote spot is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

void EventHandle::remoteMax(bool state_now, bool state_last)
{}
void df_remote_max(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote max is pressed.", __FUNCTION__, __LINE__);
	beeper.play_for_command(INVALID);
	remote.reset();
}

/* Battery */
void EventHandle::batteryHome(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::batteryLow(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
}

void EventHandle::chargeDetect(bool state_now, bool state_last)
{}
void df_charge_detect(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: default handler is called.", __FUNCTION__, __LINE__);
	if (g_charge_detect_cnt++ > 25)
	{
		ev.charge_detect = charger.getChargeStatus();
		ROS_WARN("%s %d: ev.chargeDetect has been set to %d.", __FUNCTION__, __LINE__, ev.charge_detect);
		g_charge_detect_cnt = 0;
	}
}

void EventHandle::robotSlip(bool state_new, bool state_last)
{}
void df_robot_slip(bool state_new,bool state_last)
{
	ROS_WARN("\033[32m%s,%d,set robot slip!! \033[0m",__FUNCTION__,__LINE__);
	beeper.play_for_command(true);
	g_robot_slip = true;
	g_slip_cnt ++;
}
/*
void EventHandle::lidar_bumper(bool state_new,bool state_last)
{
	g_lidar_bumper = robot::instance()->getLidarBumper();
}
*/

// Lidar stuck
void EventHandle::lidarStuck(bool state_new, bool state_last)
{}
void df_lidar_stuck(bool state_new,bool state_last)
{
	//beeper.play_for_command(true);
	//ROS_WARN("\033[32m%s %d: Lidar stuck.\033[0m", __FUNCTION__, __LINE__);
	//ev.lidarStuck = true;
}

///* Default: empty hanlder */
//void EventHandle::empty(bool state_now, bool state_last)
//{
////	ROS_INFO("%s %d: is called", __FUNCTION__, __LINE__);
//}
