//
// Created by root on 12/5/17.
//

#ifndef PP_MOVE_TYPE_HPP
#define PP_MOVE_TYPE_HPP

#include "arch.hpp"
#include "boost/shared_ptr.hpp"
#include "mode.hpp"

class Mode;
class IMoveType:public IAction
{
public:
	IMoveType();

	bool isOBSStop();
	bool isLidarStop();
	bool shouldMoveBack();
	bool shouldTurn();
//	~IMoveType() = default;

	virtual bool isFinish();
	void run();
//	bool isFinish(int& action_i);
//	virtual IAction* setNextAction()=0;
//	void registerMode(ACleanMode* sp_mode)
//	{
//		sp_mode_ = sp_mode;
//	}

	static boost::shared_ptr<IMovement> sp_movement_;
	static Mode *sp_mode_;
	static int movement_i_;
	void resetTriggeredValue();
	Point32_t start_point_;
	Point32_t target_point_;
protected:
//	Cells passed_path_;
//	Cells tmp_plan_path_;
	int16_t turn_target_angle_{};
	float back_distance_;
	enum{//movement
		mm_null,
		mm_back,
		mm_turn,
		mm_forward,
		mm_straight,
	};
};

class ActionLinear:public IMoveType
{
public:
	ActionLinear();
	~ActionLinear();
	bool isFinish() override;
//	IAction* setNextAction();
protected:
};

class ActionFollowWall:public IMoveType
{
public:
	ActionFollowWall() = delete;
	~ActionFollowWall();

	explicit ActionFollowWall(bool is_left, bool is_trapped);

	bool isFinish() override;

//	IAction* setNextAction();
	Points tmp_plan_path_{};
protected:
	bool is_left_{};
	int16_t turn_angle{};
	int16_t bumper_turn_angle(bool);
	int16_t cliff_turn_angle();
	int16_t rcon_turn_angle();
	int16_t tilt_turn_angle();
	int16_t obs_turn_angle();
	int double_scale_10(double line_angle);
	bool _lidar_turn_angle(bool is_left, int16_t& turn_angle, int lidar_min, int lidar_max, int angle_min,int angle_max,double dis_limit=0.217);
	bool lidar_turn_angle(int16_t& turn_angle);
	int16_t get_turn_angle_by_ev();
	int16_t get_turn_angle(bool);

};

class MoveTypeGoToCharger:public IMoveType
{
public:
	MoveTypeGoToCharger();

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
	bool turn_left_{true};
	int16_t turn_target_angle_{0};
	double turn_time_stamp_;
	boost::shared_ptr<IMovement> p_direct_go_movement_;
	boost::shared_ptr<IMovement> p_turn_movement_;
	boost::shared_ptr<IMovement> p_back_movement_;
};
#endif //PP_MOVE_TYPE_HPP
