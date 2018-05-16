//
// Created by root on 12/5/17.
//

#ifndef PP_MOVE_TYPE_HPP
#define PP_MOVE_TYPE_HPP
#define TRAP_IN_SMALL_AREA_COUNT 20
#define TILT_BACK_DISTANCE 0.15
//#define CLIFF_BACK_DISTANCE 0.1

#include "action.hpp"
#include "movement.hpp"
#include "boost/shared_ptr.hpp"
#include "rcon.h"
#include "serial.h"
//#include "mode.hpp"

class Mode;
class ACleanMode;
class IMoveType:public IAction
{
public:
	IMoveType();
	~IMoveType();
	bool isFinishForward();
	bool isNotHandleEvent();
	bool RconTrigger();
	bool handleMoveBackEvent(ACleanMode* p_clean_mode);
	bool handleMoveBackEventLinear(ACleanMode *p_clean_mode);
//	~IMoveType() = default;
	enum{//movement
		mm_null,
		mm_back,
		mm_turn,
		mm_stay,
		mm_rcon,
		mm_forward,
		mm_straight,
		mm_dynamic,
	};

	bool isBlockCleared(GridMap &map, Points &passed_path);
	Point_t last_{};
//	bool closed_count_{};
	void setMode(Mode* cm)
	{sp_mode_ = cm;}
	Mode* getMode()
	{return sp_mode_;}

	virtual bool isFinish();
//	void updatePath();
	void run() override;

	int8_t rcon_cnt[Rcon::enum_end + 1]{};
	uint32_t countRconTriggered(uint32_t rcon_value, int max_cnt);
	bool isRconStop();
	bool isCliffStop();
//	bool isOBSStop();
	bool isLidarStop();

	static boost::shared_ptr<IMovement> sp_movement_;
	static Mode *sp_mode_;
	static int movement_i_;
	void resetTriggeredValue();
	bool state_turn{};
//	Point_t target_point_;
	int dir_;
//	static Points remain_path_;
	bool stop_generate_next_target{};
public:
//	std::deque<double> odom_turn_target_radians_{};
	int radian_diff_count{};
	double odom_turn_target_radian_{};
	double turn_target_radian_{};
protected:

	float back_distance_;

};

class MoveTypeLinear:public IMoveType
{
public:
	MoveTypeLinear(Points remain_path);
	~MoveTypeLinear() override;
	bool isFinish() override;
//	IAction* setNextAction();

	bool isPassTargetStop(const Dir_t &dir);
	bool isCellReach();
	bool isPoseReach();

	bool isLinearForward();

private:
	void switchLinearTarget(ACleanMode * p_clean_mode);
	bool switchLinearTargetByRecalc(ACleanMode *p_clean_mode);
};

class MoveTypeFollowWall:public IMoveType
{
public:
	MoveTypeFollowWall() = delete;
	~MoveTypeFollowWall() override;

	MoveTypeFollowWall(bool is_left);
	MoveTypeFollowWall(bool is_left, bool);

	bool isFinish() override;

	bool isNewLineReach(GridMap &map);
	bool isOverOriginLine(GridMap &map);
	bool outOfRange(const Point_t &curr, Points::iterator &p_it);
	void outOfRangeFirst(bool val){
		out_of_range_state = val;
	};

	bool outOfRangeFirst(){
		return out_of_range_state;
	};

private:
    void init(bool is_left);
//    ~MoveTypeFollowWall() override;
	bool handleMoveBackEventRealTime(ACleanMode* p_clean_mode);
	bool is_left_{};
	double move_forward_time{};
	int16_t bumperTurnAngle();
	int16_t cliffTurnAngle();
	int16_t tiltTurnAngle();
	int16_t obsTurnAngle();
	bool _lidarTurnRadian(bool is_left, double &turn_radian, double lidar_min, double lidar_max, double radian_min,
						  double radian_max, bool is_moving,
						  double dis_limit = 0.3);
	bool lidarTurnRadian(double &turn_radian);
	double getTurnRadianByEvent();
	double getTurnRadian(bool);
	double robot_to_wall_distance = 0.8;
	float g_back_distance = 0.01;
	bool is_stop_follow_wall_after_tilt{};
	struct lidar_angle_param{
		double lidar_min;
		double lidar_max;
		double radian_min;
		double radian_max;
	};
//    BoundingBox<Point_t> bound_;
	bool out_of_range_state{};
};

class MoveTypeGoToCharger:public IMoveType
{
public:
	MoveTypeGoToCharger();
	~MoveTypeGoToCharger();

	bool isFinish() override ;

	void run() override ;
protected:

	boost::shared_ptr<MovementGoToCharger> p_gtc_movement_;
	boost::shared_ptr<IMovement> p_turn_movement_;
	boost::shared_ptr<IMovement> p_back_movement_;
};

class MoveTypeBumperHitTest: public IMoveType
{
public:
	MoveTypeBumperHitTest();
	~MoveTypeBumperHitTest() = default;

	bool isFinish() override;

	void run() override ;

private:
	bool bumper_error{false};
	bool turn_left_{true};
	int16_t turn_target_angle_{0};
	double turn_time_stamp_;
	boost::shared_ptr<IMovement> p_direct_go_movement_;
	boost::shared_ptr<IMovement> p_turn_movement_;
	boost::shared_ptr<IMovement> p_back_movement_;
};

class MoveTypeRemote: public IMoveType
{
public:
	MoveTypeRemote();
	~MoveTypeRemote();

	bool isFinish() override;

	void run() override ;

	// For setting the first movement of remote mode.
	static void leftStart()
	{
		start_command_ = command_type::start_left;
	}
	static void rightStart()
	{
		start_command_ = command_type::start_right;
	}
	static void forwardStart()
	{
		start_command_ = command_type::start_forward;
	}

	static int start_command_;

private:
	boost::shared_ptr<IMovement> p_movement_;

	enum command_type{
		start_null = 0,
		start_left,
		start_right,
		start_forward,
	};

	void turnLeft();
	void turnRight();
	void goForward();
	void stay();
};

class MoveTypeDeskTest: public IMoveType
{
public:
	MoveTypeDeskTest();
	~MoveTypeDeskTest() override;

	bool isFinish() override
	{
		return false;
	}

	void deskTestRoutineThread();

	void run() override;

	bool dataExtract(const uint8_t *buf);


private:
	boost::shared_ptr<IAction> p_movement_;

	uint8_t buf_[REC::REC_LEN];

	uint8_t error_step_{0}; // Marking the error stage.
	uint16_t error_content_{0}; // Marking the current value of checking item.
	uint16_t error_code_{0};

	/*
	 * Test stage: 1 ~ 7.
	 * Stage 1: Check for base value for currents.
	 * Stage 2: Check lidar, then turn 180 degrees, check lidar again, turn 180 degrees, and get the baselines
	 *          of OBS and cliff and check the rcon receivers.
	 * Stage 3: Check for bumpers and OBS value.
	 * Stage 4: Follow wall to cliff position and check for current and cliff max value.
	 * Stage 5: Check for cliffs and move back to the front of charger stub.
	 * Stage 6: Turn for 360 degrees to check for rcon.
	 */
	uint8_t test_stage_{1};

	// Each stage has several steps.
	uint8_t test_step_{0};

	// For stage 1.
	uint16_t sum_cnt_{0};

	uint32_t left_brush_current_baseline_{0};
	uint32_t right_brush_current_baseline_{0};
	uint32_t main_brush_current_baseline_{0};
	uint32_t left_wheel_current_baseline_{0};
	uint32_t right_wheel_current_baseline_{0};
	uint32_t vacuum_current_baseline_{0};
	uint32_t water_tank_current_baseline_{0};
	uint32_t robot_current_baseline_{0};

	bool checkStage1Finish();

	// For stage 2.
	uint8_t lidar_check_cnt_{0};
	double lidar_check_seq_{0};

	long left_obs_sum_{0};
	long front_obs_sum_{0};
	long right_obs_sum_{0};

	int16_t left_obs_baseline_{0};
	int16_t front_obs_baseline_{0};
	int16_t right_obs_baseline_{0};

	bool checkStage2Finish();

	// For stage 3.
	int16_t obs_ref_{2000}; //todo
	int16_t left_obs_max_{0};
	int16_t front_obs_max_{0};
	int16_t right_obs_max_{0};
	double check_start_time_{0};
	bool checkStage3Finish();

	// For stage 4.
	bool lidar_bumper_valid_{false};

	int left_wheel_current_cnt_{0};
	int right_wheel_current_cnt_{0};

	int16_t cliff_min_ref_{80}; //todo
	int16_t cliff_max_ref_{200}; //todo

	int16_t left_cliff_max_{0};
	int16_t front_cliff_max_{0};
	int16_t right_cliff_max_{0};

	long left_cliff_sum_{0};
	long front_cliff_sum_{0};
	long right_cliff_sum_{0};

	int16_t left_cliff_baseline_{0};
	int16_t front_cliff_baseline_{0};
	int16_t right_cliff_baseline_{0};

	double check_current_start_time_{0};

	uint32_t left_brush_current_{0};
	uint8_t left_brush_current_exception_cnt_{0};
	uint32_t right_brush_current_{0};
	uint8_t right_brush_current_exception_cnt_{0};
	uint32_t main_brush_current_{0};
	uint8_t main_brush_current_exception_cnt_{0};
	uint32_t left_wheel_current_{0};
	uint8_t left_wheel_current_exception_cnt_{0};
	uint32_t right_wheel_current_{0};
	uint8_t right_wheel_current_exception_cnt_{0};
	uint32_t vacuum_current_{0};
	uint32_t water_tank_current_{0};
	uint32_t robot_current_{0};

	bool checkCurrent();

	bool checkStage4Finish();

	// For stage 5.

	/*uint16_t left_brush_current_max_{0};
	uint16_t right_brush_current_max_{0};
	uint16_t main_brush_current_max_{0};
	uint16_t left_wheel_forward_current_max_{0};
	uint16_t right_wheel_forward_current_max_{0};
	uint16_t left_wheel_backward_current_max_{0};
	uint16_t right_wheel_backward_current_max_{0};
	uint16_t vacuum_current_max_{0};
	uint16_t water_tank_current_max_{0};*/

	bool checkStage5Finish();

	// For stage 6.
	bool checkStage6Finish();

	// For stage 7.
	bool checkStage7Finish();

	uint8_t charge_test_result_{0};
};

class MoveTypeGyroTest: public IMoveType
{
public:
	MoveTypeGyroTest();
	~MoveTypeGyroTest() override;

	void gyroTestRoutineThread();

	bool dataExtract(const uint8_t *buf);

	void run() override;

private:
	boost::shared_ptr<IAction> p_movement_;

	uint16_t error_code_{0};
	uint8_t error_stage_{0};
	uint16_t error_content_{0};

	uint8_t test_stage_{1};
	double last_time_stamp_{0};
	double saved_wheel_mileage_{0};
	double wheel_mileage_{0};
	double wheel_turn_angle_{0};
	double saved_gyro_turn_angle_{0};
	uint32_t count_sum{0};
};
#endif //PP_MOVE_TYPE_HPP
