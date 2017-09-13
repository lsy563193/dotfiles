//
// Created by lsy563193 on 6/28/17.
//

#ifndef PP_REGULATOR_BASE_H
#define PP_REGULATOR_BASE_H

#include <ros/ros.h>
#include <move_type.h>
#include <path_planning.h>
#include <movement.h>
#include <robot.hpp>

#define STRENGTH_WHITE_MIN 550
#define STRENGTH_WHITE_MAX 625
#define STRENGTH_BLACK_MIN 120
#define STRENGTH_BLACK_MAX 180
#define STRENGTH_HIGH_LIMIT 625
#define STRENGTH_LOW_LIMIT 150

extern int16_t g_turn_angle;
#define GO_TO_CHARGER_INIT 0
#define CHECK_NEAR_CHARGER_STATION 1
#define AWAY_FROM_CHARGER_STATION 2
#define TURN_FOR_CHARGER_SIGNAL 3
#define AROUND_CHARGER_STATION_INIT 4
#define AROUND_CHARGER_STATION 5
#define CHECK_POSITION_INIT 6
#define CHECK_POSITION 7
#define BY_PATH_INIT 8
#define BY_PATH 9

class RegulatorBase {
public:

	bool isExit();

	bool isStop(){
		return RegulatorBase::_isStop() || _isStop();
	};

	virtual bool isSwitch() = 0;
	virtual void adjustSpeed(int32_t&, int32_t&)=0;
	virtual bool _isStop()=0;
	virtual void setTarget() = 0;
	virtual bool isReach() = 0;

public:
	static Point32_t s_target;
	static Point32_t s_origin;
	static int16_t s_origin_angle;
	static int16_t s_target_angle;
	static float s_pos_x;
	static float s_pos_y;
	static Point32_t s_curr_p;
};

class BackRegulator: public RegulatorBase{
public:
	BackRegulator();
	~BackRegulator(){
		set_wheel_speed(1, 1);
	}
	void adjustSpeed(int32_t&, int32_t&);
	bool isSwitch();
	bool _isStop();
	void setTarget();

protected:
	bool isReach();

private:
	int counter_;
	int32_t speed_;
};

class TurnRegulator: public RegulatorBase{
public:

	TurnRegulator(int16_t angle);
	void adjustSpeed(int32_t&, int32_t&);
	bool isSwitch();
	bool _isStop();
	void setTarget();

protected:
	bool isReach();

private:
	uint16_t accurate_;
	uint8_t speed_;
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
		ROS_WARN("%s, %d: ", __FUNCTION__, __LINE__);
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
	bool isSwitch();
	bool _isStop();
	void setTarget(){ };
protected:
	bool isReach();

private:
	int32_t previous_;
	uint8_t seen_charger_counter;
};

class LinearRegulator: public RegulatorBase{
public:
	LinearRegulator(Point32_t, const PPTargetType&);
	~LinearRegulator(){ };
	bool _isStop();
	bool isSwitch();
	void adjustSpeed(int32_t&, int32_t&);
	void setTarget() { };

protected:
	bool isReach();

private:
	int32_t integrated_;
	int32_t base_speed_;
	uint8_t integration_cycle_;
	uint32_t tick_;
	uint8_t turn_speed_;
	PPTargetType path_;
};

class GoToChargerRegulator: public RegulatorBase{
public:
	GoToChargerRegulator();
	~GoToChargerRegulator(){ };
	bool _isStop();
	bool isSwitch();
	void adjustSpeed(int32_t&, int32_t&);
	void setTarget() { };
	void resetGoToChargerVariables()
	{
		no_signal_cnt = 0;
		move_away_from_charger_cnt = 0;
		receive_code = 0;
		current_angle = 0;
		last_angle = 0;
		angle_offset = 0;
		gyro_step = 0;
		around_charger_stub_dir = 0;
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
};

class RegulatorManage:public RegulatorBase{
public:
	RegulatorManage(const Cell_t& start_cell, const Cell_t& target, const PPTargetType& path);
	~RegulatorManage();
	void adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	bool isSwitch();
	bool _isStop();
	bool isReach();
	void mark();
	void setTarget() {p_reg_->setTarget();}

	void switchToNext();

	void updatePosition(const Point32_t &curr_point){
		s_curr_p = curr_point;
	}
private:
	RegulatorBase* p_reg_;
	RegulatorBase* mt_reg_;
	TurnRegulator* turn_reg_;
	BackRegulator* back_reg_;

};

#endif //PP_REGULATOR_BASE_H
