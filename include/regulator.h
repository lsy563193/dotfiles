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

#define WALL_DISTANCE_WHITE_MIN 550
#define WALL_DISTANCE_WHITE_MAX 625
#define WALL_DISTANCE_BLACK_MIN 120
#define WALL_DISTANCE_BLACK_MAX 180
#define WALL_DISTANCE_HIGH_LIMIT 625
#define WALL_DISTANCE_LOW_LIMIT 150

extern int16_t g_turn_angle;

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
	bool isSwitch();
	bool _isStop();
	void setTarget(){ };
protected:
	bool isReach();

private:
	int32_t previous_;
	uint8_t seen_charger_counter;
	int next_linear_speed = INT_MAX;
	double wall_follow_detect_distance=0.20;
	int32_t old_same_speed,old_diff_speed;
	int turn_right_angle_factor=15;
	int16_t wall_buffer[3]={0};
//CMMoveType last_move_type;
//bool g_is_should_follow_wall;
	//int last_strength=150;
//int last_transit_strength=150;
//double transit_time=0;
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
