//
// Created by root on 12/5/17.
//

#ifndef PP_MOVEMENT_HPP
#define PP_MOVEMENT_HPP

#include "speed_governor.hpp"
#include "event_manager.h"
#include "path_algorithm.h"
#include "boost/shared_ptr.hpp"



class IMoveType;
class IMovement: public IAction,public ISpeedGovernor{
public:
	virtual void adjustSpeed(int32_t&, int32_t&)=0;
	virtual void run();
	virtual bool isFinish()=0;
//	static ACleanMode* sp_cm_;
	static boost::shared_ptr<IMoveType> sp_mt_;
protected:
	static Point32_t s_target_p;
	static Point32_t s_origin_p;
	static float s_pos_x;
	static float s_pos_y;
//	static Path_t path_;
};

class MovementForward: public IMovement{
public:
//	MovementForward(Point32_t);
	MovementForward();
//	~MovementForward(){ };
	void adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	bool isFinish();

	bool isRconStop();
	bool isOBSStop();
	bool isLidarStop();
	bool isBoundaryStop();
	bool isPassTargetStop();
	bool isNearTarget();
	bool shouldMoveBack();
	void setTarget();
	void setBaseSpeed();

	bool isCellReach();
	bool isPoseReach();

private:

	int32_t integrated_;
	int32_t base_speed_;
	uint8_t integration_cycle_;
	uint32_t tick_;
	uint8_t turn_speed_;
////	PPTargetType path_;
	float odom_x_start;
	float odom_y_start;
};

class MovementBack: public IMovement{
public:
	MovementBack();
	~MovementBack(){
		//set_wheel.speed(1, 1);
	}
	void adjustSpeed(int32_t&, int32_t&);
	bool isLidarStop();
	void setTarget();
	bool isReach();
	bool isFinish();

private:
	int counter_;
	int32_t speed_;
	float distance;
	float lidar_detect_distance;
};

class MovementTurn: public IMovement{
public:

	MovementTurn(int16_t angle);
	void adjustSpeed(int32_t&, int32_t&);
	void setTarget();
	bool isReach();
	bool shouldMoveBack();
	bool isFinish();

private:
	uint16_t accurate_;
	uint8_t speed_;
	uint8_t stage_;
	double wait_sec_;
	double waiting_start_sec_;
	bool waiting_finished_;
	// It will use lidar points to get the turn angle for every skip_lidar_turn_angle_cnt_ times.
	uint8_t skip_lidar_turn_angle_cnt_;
};

class MovementFollowWall:public IMovement {

public:
	MovementFollowWall(bool is_left);

	~MovementFollowWall() { /*set_wheel.speed(0,0);*/ };

	bool isFinish();
	void reset_sp_turn_count() {
		turn_count = 0;
	}

	int32_t get_sp_turn_count() {
		return turn_count;
	}

	void add_sp_turn_count() {
		turn_count++;
	}

	bool sp_turn_over(const Cell_t &curr);

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

private:
//Variable for checking spot turn in wall follow mode_
	volatile int32_t turn_count;
	int32_t previous_;
	uint8_t seen_charger_counter;
	int next_linear_speed = INT_MAX;
	double wall_follow_detect_distance = 0.20;
	int32_t old_same_speed = 0;
	int32_t old_diff_speed = 0;
	int turn_right_angle_factor = 15;
	int16_t wall_buffer[3] = {0};
	bool is_right_angle = false;
	double time_right_angle = 0;
	int32_t same_speed_;
	int32_t diff_speed_;
private:
	bool is_left_{true};
};

class MovementGoToCharger: public IMovement{
public:
	MovementGoToCharger();
	~MovementGoToCharger(){ };
	bool _isStop();
	bool isSwitch();
	void adjustSpeed(int32_t&, int32_t&);
	void setTarget();
	bool isChargerReach();
	void resetGoToChargerVariables();
	bool isFinish();

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

class MovementTurnForCharger :public IMovement
{
public:
	MovementTurnForCharger();
	~MovementTurnForCharger() override ;

	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override;
	bool isFinish() override;

private:
	double start_turning_time_stamp_;
	bool turn_right_finish_;
};

class MovementStay :public IMovement
{
public:
	MovementStay();
	~MovementStay();

	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override;
	bool isFinish() override;

private:
};

class MovementDirectGo :public IMovement
{
public:
	MovementDirectGo();
	~MovementDirectGo();

	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override;
	bool isFinish() override;

private:
	double direct_go_time_stamp_;
};
#endif //PP_MOVEMENT_HPP
