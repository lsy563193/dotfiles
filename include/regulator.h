//
// Created by lsy563193 on 6/28/17.
//

#ifndef PP_REGULATOR_BASE_H
#define PP_REGULATOR_BASE_H

#include <laser.hpp>
#include <ros/ros.h>
#include <move_type.h>
#include <path_planning.h>
#include <movement.h>
#include <robot.hpp>
#define WALL_DISTANCE_WHITE_MIN 550
#define WALL_DISTANCE_WHITE_MAX 625
#define WALL_DISTANCE_BLACK_MIN 120
#define WALL_DISTANCE_BLACK_MAX 180
#define WALL_DISTANCE_HIGH_LIMIT 625
#define WALL_DISTANCE_LOW_LIMIT 150

extern int16_t g_turn_angle;
#define GO_TO_CHARGER_INIT 0
#define CHECK_NEAR_CHARGER_STATION 1
#define AWAY_FROM_CHARGER_STATION 2
#define TURN_FOR_CHARGER_SIGNAL_INIT 3
#define TURN_FOR_CHARGER_SIGNAL 4
#define AROUND_CHARGER_STATION_INIT 5
#define AROUND_CHARGER_STATION 6
#define CHECK_POSITION_INIT 7
#define CHECK_POSITION 8
#define BY_PATH_INIT 9
#define BY_PATH 10
#define TURN_CONNECT 11
#define ROUND_LEFT 1
#define ROUND_RIGHT 2

extern int g_wall_distance;
extern double g_time_straight;
extern double time_start_straight;
extern bool g_slip_backward;
extern double bumper_turn_factor;

int16_t bumper_turn_angle();

int16_t cliff_turn_angle();

int16_t tilt_turn_angle();

int16_t obs_turn_angle();

int16_t rcon_turn_angle();

bool laser_turn_angle(int16_t& turn_angle);

class RegulatorBase {
public:

	bool isExit();
	bool isStop();
	virtual void adjustSpeed(int32_t&, int32_t&)=0;
	virtual void setTarget() = 0;
	virtual std::string getName() = 0;

public:
	static Point32_t s_target_p;
	static Point32_t s_origin_p;
	static int16_t s_target_angle;
	static float s_pos_x;
	static float s_pos_y;
	static Point32_t s_curr_p;
};

class BackRegulator: public RegulatorBase{
public:
	BackRegulator();
	~BackRegulator(){
		//set_wheel_speed(1, 1);
	}
	void adjustSpeed(int32_t&, int32_t&);
	bool isLaserStop();
	void setTarget();
	bool isReach();
	std::string getName()
	{
		std::string name = "BackRegulator";
		return name;
	}

private:
	int counter_;
	int32_t speed_;
	float distance;
	float laser_detect_distance;
};

class TurnRegulator: public RegulatorBase{
public:

	TurnRegulator(int16_t angle);
	void adjustSpeed(int32_t&, int32_t&);
	void setTarget();
	bool isReach();
	bool shouldMoveBack();
	std::string getName()
	{
		std::string name = "TurnRegulator";
		return name;
	}

private:
	uint16_t accurate_;
	uint8_t speed_;
	uint8_t stage_;
	double wait_sec_;
	double waiting_start_sec_;
	bool waiting_finished_;
	// It will use laser points to get the turn angle for every skip_laser_turn_angle_cnt_ times.
	uint8_t skip_laser_turn_angle_cnt_;
};

class TurnSpeedRegulator{
public:
	TurnSpeedRegulator(uint8_t speed_max,uint8_t speed_min, uint8_t speed_start):
					speed_max_(speed_max),speed_min_(speed_min), speed_(speed_start),tick_(0){};
	~TurnSpeedRegulator() {
		//set_wheel_speed(0, 0);
	}
	bool adjustSpeed(int16_t diff, uint8_t& speed_up);

private:
	uint8_t speed_max_;
	uint8_t speed_min_;
	uint8_t speed_;
	uint32_t tick_;
};

class SelfCheckRegulator{
public:
	SelfCheckRegulator(){
		ROS_INFO("\033[33m%s\033[0m, %d: ", __FUNCTION__, __LINE__);
	};
	~SelfCheckRegulator(){
		set_wheel_speed(0, 0);
	};
	void adjustSpeed(uint8_t bumper_jam_state);
};

class FollowWallRegulator:public RegulatorBase{

public:
	FollowWallRegulator(Point32_t start_point, Point32_t target);
	~FollowWallRegulator(){ /*set_wheel_speed(0,0);*/ };
	void adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	bool shouldMoveBack();
	bool shouldTurn();
	bool isBlockCleared();
	bool isOverOriginLine();
	bool isNewLineReach();
	bool isClosure(uint8_t closure_cnt);
	bool isIsolate();
	bool isTimeUp();
	void setTarget();
	std::string getName()
	{
		std::string name = "FollowWallRegulator";
		return name;
	}

private:
	int32_t previous_;
	uint8_t seen_charger_counter;
	int next_linear_speed = INT_MAX;
	double wall_follow_detect_distance=0.20;
	int32_t old_same_speed = 0;
	int32_t old_diff_speed = 0;
	int turn_right_angle_factor=15;
	int16_t wall_buffer[3]={0};
	bool is_right_angle = false;
	double time_right_angle = 0;

	int32_t same_speed_;
	int32_t diff_speed_;
};

class LinearRegulator: public RegulatorBase{
public:
	LinearRegulator(Point32_t, const PPTargetType&);
	~LinearRegulator(){ };
	bool isRconStop();
	bool isOBSStop();
	bool isLaserStop();
	bool isBoundaryStop();
	bool isCellReach();
	bool isNearTarget();
	bool isPoseReach();
	bool shouldMoveBack();
	void adjustSpeed(int32_t&, int32_t&);
	void setTarget();
	std::string getName()
	{
		std::string name = "LinearRegulator";
		return name;
	}

private:
	int32_t integrated_;
	int32_t base_speed_;
	uint8_t integration_cycle_;
	uint32_t tick_;
	uint8_t turn_speed_;
//	PPTargetType path_;
	float odom_x_start;
	float odom_y_start;
};

class GoToChargerRegulator: public RegulatorBase{
public:
	GoToChargerRegulator();
	~GoToChargerRegulator(){ };
	bool _isStop();
	bool isSwitch();
	void adjustSpeed(int32_t&, int32_t&);
	void setTarget();
	bool isChargerReach();
	void resetGoToChargerVariables()
	{
		no_signal_cnt = 0;
		move_away_from_charger_cnt = 0;
		receive_code = 0;
		current_angle = 0;
		last_angle = robot::instance()->getAngle();
		angle_offset = 0;
		gyro_step = 0;
		around_charger_stub_dir = 0;
		go_home_bumper_cnt = 0;
		around_move_cnt = 0;
		position_far = true;
		near_counter = 0;
		side_counter = 0;
		by_path_move_cnt = 0;
		reset_rcon_status();
		turn_connect_cnt = 0;
		turn_connect_dir = ROUND_RIGHT;
	}
	std::string getName()
	{
		std::string name = "GoToChargerRegulator";
		return name;
	}

protected:
	bool isReach();

private:
	int8_t go_home_state_now;
	uint16_t no_signal_cnt;
	uint8_t move_away_from_charger_cnt;
	uint32_t receive_code;
	// This variables is for robot turning.
	float current_angle;
	float last_angle;
	float angle_offset;
	float gyro_step;
	uint8_t around_charger_stub_dir;
	uint8_t go_home_bumper_cnt;
	uint8_t check_position_dir;
	int8_t around_move_cnt;
	bool position_far;
	uint8_t near_counter;
	uint8_t side_counter;
	int8_t by_path_move_cnt;
	uint8_t turn_connect_cnt;
	uint8_t turn_connect_dir;

	int32_t left_speed_;
	int32_t right_speed_;
};

#endif //PP_REGULATOR_BASE_H
