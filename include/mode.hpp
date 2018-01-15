//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_MODE_H_H
#define PP_MODE_H_H

#include "path_algorithm.h"
#include "event_manager.h"
#include "boost/shared_ptr.hpp"
//#include "move_type.hpp"

class Mode:public EventHandle
{
public:
	virtual ~Mode() { };
	void run();

	virtual bool isExit();

	virtual bool isFinish();

	virtual void setNextMode(int next_mode);

	bool isInitState() const{
			return action_i_ == ac_open_gyro || action_i_ == ac_back_form_charger ||
		action_i_ == ac_open_lidar || action_i_	== ac_align || ac_align == ac_open_slam;
	};
	int getNextMode();

//	friend IMoveType;

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
	~ACleanMode();
	State* updateState();
	bool isFinish() override ;
	bool isExit() override;
	bool isUpdateFinish();

	void setNextModeDefault();
	virtual bool setNextAction();
	void genNextAction();

	void setRconPos(float cd,float dist);

	virtual bool mapMark() = 0;
	virtual bool MarkRealTime(){return false;};

	void setHomePoint();
	bool estimateChargerPos(uint32_t rcon_value);
	void setRconPos(Point32_t pos);

	Cells pointsGenerateCells(Points &targets);

	virtual bool MoveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt);
	virtual void actionFollowWallSaveBlocks();
	virtual void actionLinearSaveBlocks();
	void goHomePointUpdateAction();

	virtual bool MoveTypeLinearIsFinish(MoveTypeLinear *p_mt);
	int reach_cleaned_count_{};
	Points passed_path_{};
	Points plan_path_{};

	int old_dir_{};
	int new_dir_{};

	boost::shared_ptr<APathAlgorithm> clean_path_algorithm_{};
	boost::shared_ptr<GoHomePathAlgorithm> go_home_path_algorithm_{};
	GridMap clean_map_{};
	Point32_t charger_pos_{};//charger postion

	// State null
	bool checkEnterNullState();
	// State init
	virtual bool isSwitchByEventInStateInit();
	virtual bool updateActionInStateInit();
	virtual void switchInStateInit();

	// State clean
	virtual bool isSwitchByEventInStateClean();
	virtual bool updateActionInStateClean(){ return false;};
	virtual void switchInStateClean();

	// State go home point
	virtual bool checkEnterGoHomePointState();
	virtual bool isSwitchByEventInStateGoHomePoint();
	virtual bool updateActionInStateGoHomePoint();
	virtual void switchInStateGoHomePoint();

	// State go to charger
	bool checkEnterGoCharger();
	virtual bool isSwitchByEventInStateGoToCharger(){return false;};
	virtual bool updateActionInStateGoToCharger();
	virtual void switchInStateGoToCharger();

	// State exception resume
	bool checkEnterExceptionResumeState();
	virtual bool isSwitchByEventInStateExceptionResume();
	virtual bool updateActionInStateExceptionResume();
	virtual void switchInStateExceptionResume();

	// State temp spot
	virtual bool isSwitchByEventInStateSpot();
	virtual bool updateActionInStateSpot();
	virtual void switchInStateSpot(){};

	// State trapped
	virtual bool isSwitchByEventInStateTrapped();
	virtual bool updateActionInStateTrapped();
	virtual void switchInStateTrapped();
	bool trapped_time_out_{};

	// State exploration
	virtual bool isSwitchByEventInStateExploration();
	virtual bool updateActionInStateExploration();
	virtual void switchInStateExploration();

	// State resume low battery charge
	virtual bool isSwitchByEventInStateResumeLowBatteryCharge(){return false;};
	virtual bool updateActionInStateResumeLowBatteryCharge(){};
	virtual void switchInStateResumeLowBatteryCharge(){};

	// State charge
	virtual bool isSwitchByEventInStateCharge(){return false;};
	virtual bool updateActionStateCharge(){};
	virtual void switchInStateCharge(){};

	// State pause
	virtual bool isSwitchByEventInStatePause(){return false;};
	virtual bool updateActionInStatePause(){};
	virtual void switchInStatePause(){};

	void remoteHome(bool state_now, bool state_last) override ;

	void cliffAll(bool state_now, bool state_last) override ;

	// todo: Delete below 4 function.
	virtual bool isStateInitUpdateFinish(){};
	virtual bool isStateCleanUpdateFinish(){};
	virtual bool isStateGoToChargerUpdateFinish(){};

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
	bool isStateClean() const {
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
		return sp_state == state_spot;
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
	bool isStateCharge() const
	{
		return sp_state == state_charge;
	}
	bool isStatePause() const
	{
		return sp_state == state_pause;
	}
	static State *sp_state;
	static State *state_init;
	static State *state_clean;
	static State *state_exception_resume;
	static State *state_exploration;
protected:
	static std::vector<State*> sp_saved_states;
	static State *state_go_home_point;
	static State *state_go_to_charger;
	static State *state_charge;
	static State *state_trapped;
	static State *state_spot;
	static State *state_resume_low_battery_charge;
	static State *state_pause;


protected:
	bool low_battery_charge_{};
	bool moved_during_pause_{};
	Points home_points_{};
	Point32_t start_point_{0, 0, 0};
	bool should_go_to_charger_{false};
public:
//	uint8_t saveFollowWall(bool is_left);
//	std::vector<Cell_t> temp_fw_cells;
	Point32_t last_{};
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
//	void remoteHome(bool state_now, bool state_last) override ;
	void remoteDirectionLeft(bool state_now, bool state_last) override ;
//	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;
	void batteryHome(bool state_now, bool state_last) override ;
//	void overCurrentBrushLeft(bool state_now, bool state_last);
//	void overCurrentBrushMain(bool state_now, bool state_last);
//	void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
	void remoteSpot(bool state_now, bool state_last) override;
	void remoteMax(bool state_now, bool state_last) override;

//	void overCurrentSuction(bool state_now, bool state_last);

	// State init
	bool isSwitchByEventInStateInit() override;
	bool updateActionInStateInit() override;
	void switchInStateInit() override ;

	// State clean
	bool isSwitchByEventInStateClean() override;
	bool updateActionInStateClean() override;
	void switchInStateClean() override ;

	// State go home point
	bool checkEnterGoHomePointState() override;
	bool isSwitchByEventInStateGoHomePoint() override;

	// State go to charger
	bool isSwitchByEventInStateGoToCharger() override;
	void switchInStateGoToCharger() override;

	// State tmp spot
	bool checkEnterTempSpotState();
	bool checkOutOfSpot();
	bool isSwitchByEventInStateSpot() override;
	void switchInStateSpot() override;

	// State pause
	bool checkEnterPause();
	bool checkResumePause();
	bool isSwitchByEventInStatePause() override;
	bool updateActionInStatePause() override;

	// State trapped
	bool isSwitchByEventInStateTrapped() override;

	// State charge
	bool isSwitchByEventInStateCharge() override;
	bool updateActionStateCharge() override;
	void switchInStateCharge() override;

	// State resume low battery charge
	bool checkEnterResumeLowBatteryCharge();
	bool isSwitchByEventInStateResumeLowBatteryCharge() override;
	bool updateActionInStateResumeLowBatteryCharge() override;
	void switchInStateResumeLowBatteryCharge() override;

	// State exception resume
	bool isSwitchByEventInStateExceptionResume();

private:
	bool MoveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt) override;
	bool MoveTypeLinearIsFinish(MoveTypeLinear *p_mt) override;

	bool has_aligned_and_open_slam_{false};
	float paused_odom_angle_{0};
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
//	bool isExit() override;
	bool setNextAction() override;
	void keyClean(bool state_now, bool state_last) override ;
	void remoteClean(bool state_now, bool state_last) override ;
//	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;

//	void overCurrentBrushLeft(bool state_now, bool state_last);
//	void overCurrentBrushMain(bool state_now, bool state_last);
//	void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
//	void overCurrentSuction(bool state_now, bool state_last);
//	void printMapAndPath();
	void switchInStateInit() override;

	void switchInStateGoHomePoint() override;
	void switchInStateGoToCharger() override;

	bool MoveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt) override;
	bool MarkRealTime() override;
};

class CleanModeFollowWall:public ACleanMode {
public:
	CleanModeFollowWall();

	~CleanModeFollowWall() override;

	bool mapMark() override;

	void keyClean(bool state_now, bool state_last) override;
	void remoteMax(bool state_now, bool state_last) override;

	void remoteClean(bool state_now, bool state_last) override;
	void switchInStateClean() override;
	bool generatePath(GridMap &map, const Point32_t &curr, const int &last_dir, Points &targets);

	bool MoveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt) override ;

	bool updateActionInStateClean()override ;

private:
 int reach_cleaned_count_save{};
};

class CleanModeSpot:public ACleanMode
{
public:
	CleanModeSpot();
	~CleanModeSpot();

	bool mapMark() override;
//	bool isExit() override;
	bool setNextAction() override;
//	void cliffAll(bool state_now, bool state_last) override;
	void remoteClean(bool state_now, bool state_last) override;
	void keyClean(bool state_now, bool state_last) override;
	void switchInStateInit() override ;
	void switchInStateSpot() override ;
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
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
	bool updateActionInStateClean(){};

};
#endif //PP_MODE_H_H
