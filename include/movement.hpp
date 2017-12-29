//
// Created by root on 12/5/17.
//

#ifndef PP_MOVEMENT_HPP
#define PP_MOVEMENT_HPP

#include <sensor_msgs/LaserScan.h>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include "speed_governor.hpp"
#include "event_manager.h"
#include "path_algorithm.h"
#include "boost/shared_ptr.hpp"



class IMoveType;
class IMovement: public IAction,public ISpeedGovernor
{
public:
	virtual void adjustSpeed(int32_t&, int32_t&)=0;
	virtual void run();
	virtual bool isFinish()=0;
//	static ACleanMode* sp_mode_;
//	static boost::shared_ptr<IMoveType> sp_mt_;
	static IMoveType* sp_mt_;
//	static Point32_t s_target_p;
//	static Point32_t s_start_p;
protected:
	uint32_t tick_{};
	uint32_t tick_limit_{};
	static float s_pos_x;
	static float s_pos_y;
//	static Cells path_;
};

class AMovementFollowPoint:public IMovement{
public:
	virtual bool is_near()=0;
	virtual bool calcTmpTarget(Point32_t& )=0;
	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override ;

protected:

	int32_t min_speed_;
	int32_t max_speed_;
//	boost::thread* path_thread_{};
//	Point32_t tmp_target_{};
	uint8_t integration_cycle_{};
	int32_t integrated_{};
	int32_t base_speed_{};
	int kp_{20};
	const double TIME_STRAIGHT{0.2};
};

class MovementBack: public IMovement{
public:
	explicit MovementBack(float back_distance, uint8_t max_speed);
	~MovementBack(){
		//set_wheel.speed(1, 1);
	}
	void adjustSpeed(int32_t&, int32_t&);
	bool isLidarStop();
	void updateStartPose();
	bool isFinish();

private:
	uint8_t max_speed_;
	float back_distance_;
	int32_t speed_;
	uint8_t bumper_jam_cnt_;
	uint8_t cliff_jam_cnt_;
	float lidar_detect_distance;
};

class MovementTurn: public IMovement{
public:

	explicit MovementTurn(int16_t target_angle, uint8_t max_speed);
	void adjustSpeed(int32_t&, int32_t&) override;
	bool isReach();
	bool isFinish() override;

private:
	uint8_t max_speed_;
	uint16_t accurate_;
	uint8_t speed_;
	int16_t target_angle_;
};

class MovementFollowPointLinear:public AMovementFollowPoint
{
public:
//	MovementFollowPointLinear(Point32_t);
	MovementFollowPointLinear();
//	~MovementFollowPointLinear(){ };
	bool isFinish() override;

	bool is_near();
	bool isRconStop();
	bool isBoundaryStop();
	bool isPassTargetStop();
	bool isNearTarget();
//	void setTarget();
	void setBaseSpeed();

	bool isCellReach();
	bool isPoseReach();

	bool calcTmpTarget(Point32_t&) override ;

private:

	Point32_t _calcTmpTarget(const Point32_t& curr, const Point32_t& target,MapDirection new_dir);
	bool _checkIsNear(const Point32_t& curr, const Point32_t& target,MapDirection new_dir);
	uint8_t turn_speed_{};
////	PPTargetType path_;
	float odom_x_start{};
	float odom_y_start{};
	enum {
		left, fl1, fl2, fr2, fr1, right
	};
	int8_t rcon_cnt[6]{};
	int countRconTriggered(uint32_t rcon_value);
};

class IFollowWall{

public:
	explicit IFollowWall(bool is_left);

	~IFollowWall() { /*set_wheel.speed(0,0);*/ };
//	bool isFinish();
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

	bool isBlockCleared();

//	bool isClosure(uint8_t closure_cnt);

	bool isIsolate();

//	bool isTimeUp();

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
protected:
	bool is_left_{true};
};

class MovementFollowWallLidar:public AMovementFollowPoint, public IFollowWall
{

public:
	explicit MovementFollowWallLidar(bool is_left);

	bool calcTmpTarget(Point32_t&) override ;

	bool isFinish() override ;
	bool is_near();
private:
//	Point32_t temp_target{};

	bool is_sp_turn{};
	uint32_t seq_{0};
};

class MovementGoToCharger: public IMovement
{
public:
	MovementGoToCharger();
	~MovementGoToCharger() = default;
	bool _isStop();
	bool isSwitch();
	void adjustSpeed(int32_t&, int32_t&) override ;
	void getTurnBackInfo(int16_t &turn_angle, float &back_distance);
	bool isFinish() override ;

private:
	void resetGoToChargerVariables();
	int8_t gtc_state_now_;
	enum{
		gtc_init,
		gtc_check_near_charger_station,
		gtc_away_from_charger_station,
		gtc_turn_for_charger_signal_init,
		gtc_turn_for_charger_signal,
		gtc_around_charger_station_init,
		gtc_around_charger_station,
		gtc_check_position_init,
		gtc_check_position,
		gtc_by_path_init,
		gtc_by_path,
		gtc_turn_for_charger
	};
	enum{
		gtc_check_position_left,
		gtc_check_position_right

	};
	int16_t turn_angle_;
	float back_distance_;
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

class MovementExceptionResume: public IMovement
{
public:
	MovementExceptionResume();
	~MovementExceptionResume();

	void adjustSpeed(int32_t&, int32_t&) override ;
	bool isFinish() override;

private:
	double resume_wheel_start_time_;
	float wheel_current_sum_;
	uint8_t wheel_current_sum_cnt_;
	uint8_t wheel_resume_cnt_;
	uint8_t bumper_jam_state_;
};

class MovementCharge :public IMovement
{
public:
	MovementCharge();
	~MovementCharge() override ;

	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override;
	bool isFinish() override;
	void run() override;

private:
	bool directly_charge_{false};
	uint8_t disconnect_charger_count_{0};
	time_t show_battery_info_time_stamp_;
	bool turn_for_charger_{false};
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

class MovementStraight :public IMovement
{
public:
	MovementStraight();
	~MovementStraight();

	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override;
	bool isFinish() override;

private:
//	double direct_go_time_stamp_;
};
class MovementForwardTurn :public IMovement
{
public:
	MovementForwardTurn() = delete;

	explicit MovementForwardTurn(bool);
	~MovementForwardTurn() = default;

	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override;
	bool isFinish() override;

private:
	bool is_left_;
//	double direct_go_time_stamp_;
};

#endif //PP_MOVEMENT_HPP
