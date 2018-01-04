//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_MODE_H_H
#define PP_MODE_H_H

#include "path_algorithm.h"
#include "event_manager.h"
#include "boost/shared_ptr.hpp"
#include "move_type.hpp"

class Mode:public EventHandle
{
public:
	virtual ~Mode() { };
	void run();

	virtual bool isExit();

	virtual bool isFinish();

	virtual void setNextMode(int next_mode);

	int getNextMode();

	friend IMoveType;

	enum {
		md_idle,
		md_charge,
		md_sleep,
		md_go_to_charger,
		md_remote,

		cm_navigation,
		cm_wall_follow,
		cm_spot,
		cm_exploration,

		cm_test,
	};

	int next_mode_i_;

	int action_i_{ac_null};
	enum {
		//0
		ac_null,
		ac_open_gyro,
		ac_back_form_charger,
		ac_open_lidar,
		ac_align,
		//5
		ac_open_slam,
		ac_linear,
		ac_follow_wall_left,
		ac_follow_wall_right,
		ac_turn,
		//10
		ac_back,
		ac_go_to_charger,
		ac_idle,
		ac_sleep,
		//15
		ac_charge,
		ac_movement_stay,
		ac_movement_direct_go,
		ac_pause,
		//20
		ac_exception_resume,
		ac_check_bumper,
		ac_check_vacuum,
		ac_bumper_hit_test,
	};

	bool isExceptionTriggered();

	static boost::shared_ptr<IAction> sp_action_;
protected:

	int mode_i_{ac_null};

private:

};

class ModeIdle:public Mode
{
public:
	ModeIdle();
	~ModeIdle() override;
	bool isExit() override;
	bool isFinish() override;
	void remoteKeyHandler(bool state_now, bool state_last);
	void remoteDirectionLeft(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteDirectionRight(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteDirectionForward(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteHome(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteSpot(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteWallFollow(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteClean(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteMax(bool state_now, bool state_last) override ;
	void remotePlan(bool state_now, bool state_last) override ;
	void keyClean(bool state_now, bool state_last) override;
	void chargeDetect(bool state_now, bool state_last) override ;
	void rcon(bool state_now, bool state_last) override ;

protected:
//	std::vector<Cell_t> temp_fw_cells;
private:
	void register_events(void);
	bool battery_low_{false};

	bool plan_activated_status_;

	/*---values for rcon handle---*/
	double first_time_seen_charger_;
	double last_time_seen_charger_;
};

class ModeSleep: public Mode
{
public:
	ModeSleep();
	~ModeSleep() override ;

	bool isExit() override ;
	bool isFinish() override ;

	IAction* getNextAction();

	// For exit event handling.
	void remoteClean(bool state_now, bool state_last) override;
	void keyClean(bool state_now, bool state_last) override;
	void chargeDetect(bool state_now, bool state_last) override;
	void rcon(bool state_now, bool state_last) override;
	void remotePlan(bool state_now, bool state_last) override;

private:
	bool plan_activated_status_;
};

class ModeCharge: public Mode
{
public:
	ModeCharge();
	~ModeCharge() override ;

	bool isExit() override ;
	bool isFinish() override ;

	// For exit event handling.
	void remoteClean(bool state_now, bool state_last) override ;
	void keyClean(bool state_now, bool state_last) override ;
	void remotePlan(bool state_now, bool state_last) override ;

private:
	bool plan_activated_status_;
};

class ModeRemote: public Mode
{
public:
	ModeRemote();
	~ModeRemote() override ;

	bool isExit() override ;
	bool isFinish() override ;

	IAction* getNextAction();

	// For exit event handling.
	void remoteClean(bool state_now, bool state_last) override ;
	void remoteDirectionLeft(bool state_now, bool state_last) override ;
	void remoteDirectionRight(bool state_now, bool state_last) override ;
	void remoteDirectionForward(bool state_now, bool state_last) override ;
	void keyClean(bool state_now, bool state_last) override ;

private:
	double remote_mode_time_stamp_;

};

class ModeGoToCharger: public Mode
{
public:
	ModeGoToCharger();
	~ModeGoToCharger();

	bool isExit() override;
	bool isFinish() override;

	IAction* getNextAction();

	void keyClean(bool state_now, bool state_last) override ;
	void remoteClean(bool state_now, bool state_last) override ;
	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;
//void overCurrentBrushLeft(bool state_now, bool state_last);
//void overCurrentBrushMain(bool state_now, bool state_last);
//void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
//	void overCurrentSuction(bool state_now, bool state_last);

};

class State;
class MoveTypeFollowWall;
class MoveTypeLinear;
class ACleanMode:public Mode
{
public:
	ACleanMode();
	bool isFinish() override ;
	bool isExit() override;
	void setNextModeDefault();
	bool setNextState();
	virtual bool setNextAction();
	void genNextAction();

	bool setNextStateForGoHomePoint(GridMap &map);
	void setRconPos(float cd,float dist);

	virtual bool mapMark() = 0;
	/*
	 * @author mengshige1988@qq.com
	 * @breif estimate charge postiion ,according to rcon sensor signals
	 * @return true if found ,else false
	 * */
	bool estimateChargerPos(uint32_t rcon_value);

	Cells pointsGenerateCells(Points &targets);

	virtual bool actionFollowWallIsFinish(MoveTypeFollowWall *p_mt);
	virtual void actionFollowWallSaveBlocks();
	virtual bool actionLinearIsFinish(MoveTypeLinear *p_mt);
	void setRconPos(Point32_t pos);
	Point32_t updatePath(GridMap& map);
	int reach_cleaned_count_{};
	static Points passed_path_;
	static Points plan_path_;

	MapDirection old_dir_{MAP_POS_X};
	MapDirection new_dir_{MAP_POS_X};

	boost::shared_ptr<APathAlgorithm> clean_path_algorithm_{};
	boost::shared_ptr<GoHomePathAlgorithm> go_home_path_algorithm_{};
	GridMap clean_map_;
	Point32_t charger_pos_{};//charger postion

	virtual bool isFinishInit(){return false;};
	virtual bool isFinishClean(){return false;};
	virtual bool isFinishGoHomePoint(){return false;};
	virtual bool isFinishGoCharger(){return false;};
	virtual bool isFinishTmpSpot(){return false;};
	virtual bool isFinishTrapped(){ return false;};
	virtual bool isFinishExceptionResume(){return false;};
	virtual bool isFinishExploration(){ return false;};
	virtual bool isFinishResumeLowBatteryCharge(){return false;};
	virtual bool isFinishLowBatteryResume(){return false;};
	virtual bool isFinishCharge(){return false;};
	virtual bool isFinishPause(){return false;};

public:
	State* getState() const {
		return sp_state;
	};
	void setState(State* state){
		sp_state = state;
	}
	bool isStateInit() const
	{
		return sp_state == state_init;
	}

	bool isStateClean() const
	{
		return sp_state == state_clean;
	}
	bool isStateGoHomePoint() const
	{
		return sp_state == state_go_home_point;
	}
	bool isStateGoCharger() const
	{
		return sp_state == state_go_to_charger;
	}
	bool isStateTrapped() const
	{
		return sp_state == state_trapped;
	}
	bool isStateTmpSpot() const
	{
		return sp_state == state_tmp_spot;
	}
	bool isStateExceptionResume() const
	{
		return sp_state == state_exception_resume;
	}
	bool isStateExploration() const
	{
		return sp_state == state_exploration;
	}
	bool isStateResumeLowBatteryCharge() const
	{
		return sp_state == state_resume_low_battery_charge;
	}
	bool isStatePause() const
	{
		return sp_state == state_pause;
	}
protected:
	static State *sp_state;
	static State *state_saved_state_before_pause;
	static State *state_init;
	static State *state_clean;
	static State *state_go_home_point;
	static State *state_go_to_charger;
	static State *state_charge;
	static State *state_trapped;
	static State *state_tmp_spot;
	static State *state_exception_resume;
	static State *state_exploration;
	static State *state_resume_low_battery_charge;
	static State *state_pause;

	HomePoints home_points_;
	bool reach_home_point_{false};
public:
//	uint8_t saveFollowWall(bool is_left);
//	std::vector<Cell_t> temp_fw_cells;
	Point32_t last_;
	bool found_temp_charger_{};
	bool in_rcon_signal_range_{};
	bool should_mark_charger_{};
	bool should_mark_temp_charger_{};
	bool found_charger_{};
};

class CleanModeNav:public ACleanMode
{
public:
	CleanModeNav();
	~CleanModeNav();

	bool mapMark() override ;
	bool isExit() override;

	bool setNextAction() override;
	void keyClean(bool state_now, bool state_last) override ;
	void remoteClean(bool state_now, bool state_last) override ;
	void remoteHome(bool state_now, bool state_last) override ;
	void remoteDirectionLeft(bool state_now, bool state_last) override ;
	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;
	void batteryHome(bool state_now, bool state_last) override ;
//	void overCurrentBrushLeft(bool state_now, bool state_last);
//	void overCurrentBrushMain(bool state_now, bool state_last);
//	void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
	void remoteSpot(bool state_now, bool state_last) override;
//	void overCurrentSuction(bool state_now, bool state_last);


	bool isFinishInit() override;
	bool isFinishClean() override;
	bool isFinishGoHomePoint() override;
	bool isFinishGoCharger() override;
	bool isFinishTmpSpot() override;
	bool isFinishTrapped() override;
	bool isFinishExceptionResume() override;
	bool isFinishExploration() override;
	bool isFinishResumeLowBatteryCharge() override;
	bool isFinishLowBatteryResume() override;
	bool isFinishCharge() override;
	bool isFinishPause() override;

private:
	bool actionFollowWallIsFinish(MoveTypeFollowWall *p_mt) override;
	bool actionLinearIsFinish(MoveTypeLinear *p_mt) override;
	void actionFollowWallSaveBlocks() override ;
	void enterPause();
	void resumePause();
	void resumeLowBatteryCharge();
	void switchToGoHomePointState();

	bool low_battery_charge_{false};
	bool has_aligned_and_open_slam_{false};
	float paused_odom_angle_{0};
	bool moved_during_pause_;
	Point32_t continue_point_{};
	bool go_home_for_low_battery_{false};

protected:
//	Cells home_point_{};
public:

};

class CleanModeExploration : public ACleanMode
{
public:
	CleanModeExploration();
	~CleanModeExploration();

	bool mapMark() override;
	bool isExit() override;
	bool setNextAction() override;
	void keyClean(bool state_now, bool state_last) override ;
	void remoteClean(bool state_now, bool state_last) override ;
	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;
//	void overCurrentBrushLeft(bool state_now, bool state_last);
//	void overCurrentBrushMain(bool state_now, bool state_last);
//	void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
//	void overCurrentSuction(bool state_now, bool state_last);
	void printMapAndPath();

	bool isFinishInit() override;
	bool isFinishClean() override;
	bool isFinishGoHomePoint() override;
	bool isFinishGoCharger() override;

};

class CleanModeFollowWall:public ACleanMode {
public:
	CleanModeFollowWall();

	~CleanModeFollowWall() override;

	bool actionFollowWallIsFinish(MoveTypeFollowWall *p_mt) override;

	bool setNextAction() override;

	bool mapMark() override;

	void keyClean(bool state_now, bool state_last) override;

//	void overCurrentWheelLeft(bool state_now, bool state_last);
//
//	void overCurrentWheelRight(bool state_now, bool state_last);
//
	void remoteClean(bool state_now, bool state_last) override;
//
//	void remoteHome(bool state_now, bool state_last);
//
//	void remoteDirectionLeft(bool state_now, bool state_last);
//
//	void cliffAll(bool state_now, bool state_last);
//
//	void batteryHome(bool state_now, bool state_last);
//
//	void chargeDetect(bool state_now, bool state_last);

	int16_t wf_path_find_shortest_path(GridMap& map, int16_t xID, int16_t yID, int16_t endx, int16_t endy, uint8_t bound);

	int16_t wf_path_find_shortest_path_ranged(GridMap& map, int16_t curr_x, int16_t curr_y, int16_t end_x, int16_t end_y, uint8_t bound,
																						int16_t x_min, int16_t x_max, int16_t y_min, int16_t y_max,
																						bool used_unknown);
	bool wf_is_isolate(GridMap& map);


	bool isFinishInit() override;
	bool isFinishClean() override;
	bool isFinishGoHomePoint() override;
	bool isFinishGoCharger() override;
private:
	uint32_t diff_timer_;
protected:
//	Cells home_point_{};
private:

};

class CleanModeSpot:public ACleanMode
{
public:
	CleanModeSpot();
	~CleanModeSpot();

	bool mapMark() override;
	bool isExit() override;
	bool setNextAction() override;
	void cliffAll(bool state_now, bool state_last) override;
	void remoteClean(bool state_now, bool state_last) override;
	void keyClean(bool state_now, bool state_last) override;

	bool isFinishInit() override;
	bool isFinishClean() override;
private:

};

class CleanModeTest:public ACleanMode
{
public:
	CleanModeTest();
	~CleanModeTest();

	bool mapMark() override;

	bool isFinish() override;

	bool setNextAction() override;
	void keyClean(bool state_now, bool state_last) override ;
	void remoteMax(bool state_now, bool state_last) override ;
	void remoteDirectionForward(bool state_now, bool state_last) override ;

};
#endif //PP_MODE_H_H
