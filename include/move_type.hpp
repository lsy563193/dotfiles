//
// Created by root on 12/5/17.
//

#ifndef PP_MOVE_TYPE_HPP
#define PP_MOVE_TYPE_HPP

//#include "arch.hpp"
#include "boost/shared_ptr.hpp"

class ACleanMode;
class IMoveType:public IAction
{
public:
	IMoveType();
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
	static boost::shared_ptr<ACleanMode> sp_cm_;
	static int movement_i_;
	void resetTriggeredValue();
	Point32_t start_point_;
	Point32_t target_point_;
protected:
//	Path_t passed_path_;
//	Path_t plan_path_;
	int16_t turn_target_angle_{};
	float back_distance_;
	enum{//movement
		mm_null,
		mm_back,
		mm_turn,
		mm_forward,
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

	explicit ActionFollowWall(bool is_left);

	bool isFinish() override;

//	IAction* setNextAction();
protected:
	bool is_left_{};
	int16_t turn_angle{};
	int16_t bumper_turn_angle();
	int16_t cliff_turn_angle();
	int16_t tilt_turn_angle();
	int16_t obs_turn_angle();
	int16_t rcon_turn_angle();
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

	int state_;
	enum {
		gtc_back_,
		gtc_turn_,
		gtc_,
	};
	boost::shared_ptr<MovementGoToCharger> sp_gtc_movement_;
	boost::shared_ptr<IMovement> sp_turn_movement_;
	boost::shared_ptr<IMovement> sp_back_movement_;
};

#endif //PP_MOVE_TYPE_HPP
