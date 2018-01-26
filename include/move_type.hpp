//
// Created by root on 12/5/17.
//

#ifndef PP_MOVE_TYPE_HPP
#define PP_MOVE_TYPE_HPP

#include "action.hpp"
#include "movement.hpp"
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
	bool RconTrigger();
//	~IMoveType() = default;

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

	enum {
		left, fl1, fl2, fr2, fr1, right
	};
	int8_t rcon_cnt[6]{};
	int countRconTriggered(uint32_t rcon_value);
	bool isRconStop(bool back_from_charger,Cell_t charge_pose);
	bool isOBSStop();
	bool isLidarStop();

	static boost::shared_ptr<IMovement> sp_movement_;
	static Mode *sp_mode_;
	static int movement_i_;
	void resetTriggeredValue();
	Point_t start_point_;
	bool state_turn{};
//	Point_t target_point_;
	int dir_;
protected:
//	Cells passed_path_;
//	Cells tmp_plan_path_;
	double turn_target_radian_{};

	float back_distance_;
	enum{//movement
		mm_null,
		mm_back,
		mm_turn,
		mm_stay,
		mm_rcon,
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

	bool isPassTargetStop(Dir_t &dir);
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

private:
	bool handleMoveBackEvent(ACleanMode* p_clean_mode);
	bool is_left_{};
	int16_t bumperTurnAngle();
	int16_t cliffTurnAngle();
	int16_t rconTurnAngle();
	int16_t tiltTurnAngle();
	int16_t obsTurnAngle();
	int16_t double_scale_10(double line_angle);
	bool _lidarTurnRadian(bool is_left, double &turn_radian, double lidar_min, double lidar_max, double radian_min,
						  double radian_max,
						  double dis_limit = 0.217);
	bool lidarTurnRadian(double &turn_radian);
	double getTurnRadianByEvent();
	double getTurnRadian(bool);
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

class MoveTypeRemote: public IMoveType
{
public:
	MoveTypeRemote();
	~MoveTypeRemote() override;

	bool isFinish() override;

	void run() override ;

private:
	boost::shared_ptr<IMovement> p_movement_;
};
#endif //PP_MOVE_TYPE_HPP
