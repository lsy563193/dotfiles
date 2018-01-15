//
// Created by root on 12/5/17.
//

#ifndef PP_MOVE_TYPE_HPP
#define PP_MOVE_TYPE_HPP

//#include "arch.hpp"
#include "boost/shared_ptr.hpp"
//#include "mode.hpp"

class Mode;
class ACleanMode;
class IMoveType:public IAction
{
public:
	IMoveType();

	bool shouldMoveBack();
	bool shouldTurn();
//	~IMoveType() = default;

	void setMode(Mode* cm)
	{sp_mode_ = cm;}
	Mode* getMode()
	{return sp_mode_;}

	virtual bool isFinish();
	void updatePath();
	void run() override;

	enum {
		left, fl1, fl2, fr2, fr1, right
	};
	int8_t rcon_cnt[6]{};
	int countRconTriggered(uint32_t rcon_value);
	bool isRconStop();
	bool isOBSStop();
	bool isLidarStop();

	static boost::shared_ptr<IMovement> sp_movement_;
	static Mode *sp_mode_;
	static int movement_i_;
	void resetTriggeredValue();
	Point32_t start_point_;
	bool state_turn{};
//	Point32_t target_point_;
	int dir_;
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

class MoveTypeLinear:public IMoveType
{
public:
	MoveTypeLinear();
	~MoveTypeLinear() override;
	bool isFinish() override;
//	IAction* setNextAction();

	bool isPassTargetStop(int &dir);
	bool isCellReach();
	bool isPoseReach();

	bool isLinearForward();

private:
	bool handleMoveBackEvent(ACleanMode* p_clean_mode);
	void switchLinearTarget(ACleanMode * p_clean_mode);
};

class MoveTypeFollowWall:public IMoveType
{
public:
	MoveTypeFollowWall() = delete;
	~MoveTypeFollowWall() override;

	explicit MoveTypeFollowWall(bool is_left);

	bool isFinish() override;

	bool isNewLineReach(GridMap &map);
	bool isOverOriginLine(GridMap &map);
	bool isBlockCleared(GridMap &map, Points &passed_path);

private:
	bool handleMoveBackEvent(ACleanMode* p_clean_mode);
	bool is_left_{};
	int16_t turn_angle{};
	int16_t bumperTurnAngle();
	int16_t cliffTurnAngle();
	int16_t rconTurnAngle();
	int16_t tiltTurnAngle();
	int16_t obsTurnAngle();
	int double_scale_10(double line_angle);
	bool _lidarTurnAngle(bool is_left, int16_t &turn_angle, int lidar_min, int lidar_max, int angle_min, int angle_max,
						 double dis_limit = 0.217);
	bool lidarTurnAngle(int16_t &turn_angle);
	int16_t getTurnAngleByEvent();
	int16_t getTurnAngle(bool);
	double robot_to_wall_distance = 0.8;
	float g_back_distance = 0.01;
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
