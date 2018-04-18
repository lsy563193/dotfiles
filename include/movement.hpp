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

#include "action.hpp"

class IMoveType;
class IMovement: public IAction,public ISpeedGovernor
{
public:
//	virtual void adjustSpeed(int32_t&, int32_t&)=0;
	void run() override;
	virtual bool isFinish()=0;
//	static ACleanMode* sp_mode_;
//	static boost::shared_ptr<IMoveType> sp_mt_;
	static IMoveType* sp_mt_;
//	static Point_t s_target_p;
//	static Point_t s_start_p;
protected:
	uint32_t tick_{};
	uint32_t tick_limit_{};
	static float s_pos_x;
	static float s_pos_y;
//	static Cells path_;
};

class AMovementFollowPoint:public IMovement{
public:
	virtual uint8_t isNear()=0;
	virtual Point_t calcTmpTarget()=0;
	bool isFinish() override ;
	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override ;
	double radian_diff{};
	void getLRSpeed(int32_t&, int32_t&);

protected:

	bool is_out_corner{};
	int32_t min_speed_;
	int32_t max_speed_;
//	static Point_t tmp_target_;
//	boost::thread* path_thread_{};
//	Point_t tmp_target_{};
	uint8_t integration_cycle_{};
	int32_t integrated_{};
	int32_t base_speed_{};
	int32_t left_speed_{};
	int32_t right_speed_{};
	double angle_forward_to_turn_{};
	int kp_{2};
//	const double TIME_STRAIGHT{0.2};
};

class MovementBack: public IMovement{
public:
	explicit MovementBack(float back_distance, uint8_t max_speed);
	~MovementBack();
	void adjustSpeed(int32_t&, int32_t&) override;
	bool isLidarStop();
	void updateStartPose();
	bool isFinish() override;

private:
	uint8_t max_speed_;
	float back_distance_;
	int32_t speed_;
	uint8_t bumper_jam_cnt_;
	uint8_t lidar_bumper_jam_cnt_;
	uint8_t cliff_jam_cnt_;
	uint8_t robot_stuck_cnt_;
	uint8_t cliff_status_{0};
	float lidar_detect_distance;
};

class MovementGyroDynamic : public IMovement{
public:
	MovementGyroDynamic();
	~MovementGyroDynamic();
	void adjustSpeed(int32_t&, int32_t&) override;
	bool isFinish() override;
private:
	bool is_open_dynamic_succeed_{};
	double start_dynamic_time_{};
};

class MovementRcon: public IMovement
{
public:
	MovementRcon(bool is_left);
	~MovementRcon();
	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override ;
	bool isFinish() override;

private:
	uint32_t rcon_status;
	int32_t left_speed_{};
	int32_t right_speed_{};
	uint8_t seen_charger_counter_{};
	bool	is_left_;
};

class MovementTurn: public IMovement{
public:

	explicit MovementTurn(double radian, uint8_t max_speed);
	void adjustSpeed(int32_t&, int32_t&) override;
	bool isReach();
	bool isFinish() override;

private:
	double turn_radian_;
	uint8_t max_speed_;
	double accurate_;
	uint8_t speed_;
	double target_radian_;
};

class MovementFollowPointLinear:public AMovementFollowPoint
{
public:
//	MovementFollowPointLinear(Point_t);
	MovementFollowPointLinear();

//	void scaleCorrectionPos();
//	~MovementFollowPointLinear(){ };
	bool isFinish() override;
	/**
	* @brief judge is the robot is close to the obstacle and speed down
	*
	* @return
	*        -<em>0</em> no obstacle
	*        -<em>1</em> exist obstacle, speed down level is 1, for obs and laser
	*        -<em>2</em> exist obstacle, speed down level is 2, for slam map
	*/
	uint8_t isNear() override;
//	void setTarget();

	void scaleCorrectionPos(Point_t &tmp_pos);
	Point_t calcTmpTarget() override ;
	Point_t _calcTmpTarget();
//	Point_t tmp_pos;
//	bool calcTmpTarget() override ;
//	Point_t _calcTmpTargetNoneRealTime();
//	Point_t _calcTmpTargetRealTime();

private:
};

class IFollowWall{

public:
	explicit IFollowWall(bool is_left);

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

//	bool isClosure(uint8_t closure_cnt);

	bool isIsolate();

//	bool isTimeUp();

	void setTarget();

private:
//Variable for checking spot turn in wall follow is_max_clean_state_
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

	Point_t calcTmpTarget() override ;//laser follow wall algorithm
	Points calcVirtualTmpTarget();//generate a circle path

	bool isFinish() override ;
	uint8_t isNear() override ;
private:
	Points virtual_targets_{};
	Points lidar_targets_{};
	Points* p_tmp_targets_{};

	uint32_t seq_{0};
	ros::Time corner_time;
};

class MovementGoToCharger: public IMovement
{
public:
	MovementGoToCharger();
	~MovementGoToCharger() = default;
	bool _isStop();
	bool isSwitch();
	void adjustSpeed(int32_t&, int32_t&) override ;
	void getTurnBackInfo(double &turn_radian, float &back_distance);
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
	double turn_angle_;
	float back_distance_;
	uint16_t no_signal_cnt;
	double move_away_from_charger_time_stamp_;
	uint32_t receive_code;
	// This variables is for robot turning.
	double current_radian_;
	double last_radian_;
	double radian_offset_;
	float gyro_radian_step_;
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
	double wheel_cliff_start_time_;
	uint8_t oc_main_brush_cnt_{0};
	uint8_t main_brush_resume_state_{1};
	double resume_main_bursh_start_time_;
	uint8_t oc_vacuum_resume_cnt_{0};
	double resume_vacuum_start_time_;
	double resume_lidar_start_time_;
	uint8_t wheel_resume_cnt_{0};
	uint8_t bumper_jam_state_{1};
	uint8_t lidar_bumper_jam_state_{1};
	double bumper_resume_start_radian_{0};
	double wheel_cliff_resume_start_radian_{0};
	uint8_t robot_slip_flag_{0};
	static double slip_start_turn_time_;
	static bool is_slip_last_turn_left_;
	double resume_slip_start_time_;
	uint8_t cliff_resume_cnt_{0};
	uint8_t cliff_all_resume_cnt_{0};
	uint8_t wheel_cliff_resume_cnt_{0};
	uint8_t wheel_cliff_state_{1};
	uint8_t robot_stuck_resume_cnt_{0};
	uint8_t lidar_resume_cnt_{0};
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
	double battery_full_start_time_{0};
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
	MovementStay(double stay_time_sec);
	~MovementStay();

	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override;
	bool isFinish() override;
	int bumper_status_in_stay_ = 0;
	int cliff_status_in_stay_ = 0;
	int tilt_status_in_stay_ = 0;
private:
};

class MovementStayRemote :public MovementStay{
public:
	MovementStayRemote(double stay_time_sec);
	bool isFinish() override;
};
class MovementDirectGo :public IMovement
{
public:
	MovementDirectGo(bool slow_down, float timeout = 17); // 17s for 5 meters straight movement.
	~MovementDirectGo();

	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override;
	bool isFinish() override;

private:
	bool slow_down_{false};
	int16_t speed_{LINEAR_MIN_SPEED};
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
