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
	/*
	 * @author mengshige1988@qq.com
	 * @brief estimate charge position ,according to rcon sensor signals
	 * @return true if found ,else false
	 * */
	bool estimateChargerPos(uint32_t rcon_value);
	void setRconPos(Point32_t pos);

	Cells pointsGenerateCells(Points &targets);

	virtual bool actionFollowWallIsFinish(MoveTypeFollowWall *p_mt);
	virtual void actionFollowWallSaveBlocks();
	void goHomePointUpdateAction();

	virtual bool actionLinearIsFinish(MoveTypeLinear *p_mt);
	int reach_cleaned_count_{};
	static Points passed_path_;
	static Points plan_path_;

	static int old_dir_;
	static int new_dir_;

	static boost::shared_ptr<APathAlgorithm> clean_path_algorithm_;
	static boost::shared_ptr<GoHomePathAlgorithm> go_home_path_algorithm_;
	static GridMap clean_map_;
	Point32_t charger_pos_{};//charger postion

	// State null
	bool checkEnterNullState();
	// State init
	virtual bool isSwitchByEventInStateInit();
	virtual bool updateActionInStateInit();
	virtual void switchInStateInit();

	// State clean
	virtual bool isSwitchByEventInStateClean();
	virtual bool updateActionInStateClean() = 0;
	virtual void switchInStateClean();

	// State go home point
	virtual bool checkEnterGoHomePointState();
	virtual bool isSwitchByEventInStateGoHomePoint();
	virtual bool updateActionInStateGoHomePoint();
	virtual void switchInStateGoHomePoint();

	// State go to charger
	virtual bool isSwitchByEventInStateGoToCharger(){return false;};
	virtual bool updateActionInStateGoToCharger();
	virtual void switchInStateGoToCharger();

	// State exception resume
	bool checkEnterExceptionResumeState();
	virtual bool isSwitchByEventInStateExceptionResume(){return false;};
	virtual bool updateActionInStateExceptionResume(){};
	virtual void switchInStateExceptionResume(){};

	// State temp spot
	bool updateActionSpot();
	virtual bool isSwitchByEventInStateTmpSpot(){return false;};
	virtual bool updateActionInStateTmpSpot();
	virtual void switchInStateTmpSpot(){};

	// State trapped
	virtual bool isSwitchByEventInStateTrapped(){ return false;};
	virtual bool updateActionInStateTrapped(){};
	virtual void switchInStateTrapped(){ };

	// State exploration
	virtual bool isSwitchByEventInStateExploration(){ return false;};
	virtual bool updateActionInStateExploration(){};
	virtual void switchInStateExploration(){ };

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

	// todo: Delete below 4 function.
	virtual bool isStateInitUpdateFinish(){};
	virtual bool isStateCleanUpdateFinish(){};
	virtual bool isStateGoHomePointUpdateFinish();
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
	bool isStateCharge() const
	{
		return sp_state == state_charge;
	}
	bool isStatePause() const
	{
		return sp_state == state_pause;
	}
	static State *sp_state;
	static State *state_clean;
protected:
	static State *sp_saved_state;
	static State *state_init;
	static State *state_go_home_point;
	static State *state_go_to_charger;
	static State *state_charge;
	static State *state_trapped;
	static State *state_tmp_spot;
	static State *state_exception_resume;
	static State *state_exploration;
	static State *state_resume_low_battery_charge;
	static State *state_pause;


protected:
	static bool low_battery_charge_;
	static bool moved_during_pause_;
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
	bool updateActionInStateGoHomePoint() override;
	void switchInStateGoHomePoint() override ;

	// State go to charger
	bool isSwitchByEventInStateGoToCharger() override;
	void switchInStateGoToCharger() override;

	// State tmp spot
    bool isSwitchByEventInStateTmpSpot() override;
//    bool updateActionInStateTmpSpot() override ;
    void switchInStateTmpSpot() override;

	// State pause
	bool checkEnterPause();
	bool checkResumePause();
	bool isSwitchByEventInStatePause() override;
	bool updateActionInStatePause() override;

private:
	bool actionFollowWallIsFinish(MoveTypeFollowWall *p_mt) override;
	void actionFollowWallSaveBlocks() override ;
	bool actionLinearIsFinish(MoveTypeLinear *p_mt);
	void resumeLowBatteryCharge();
	bool checkEnterTempSpotState();

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
	bool isExit() override;
	bool setNextAction() override;
	void keyClean(bool state_now, bool state_last) override ;
	void remoteClean(bool state_now, bool state_last) override ;
	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;

	bool updateActionInStateClean(){};
//	void overCurrentBrushLeft(bool state_now, bool state_last);
//	void overCurrentBrushMain(bool state_now, bool state_last);
//	void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
//	void overCurrentSuction(bool state_now, bool state_last);
	void printMapAndPath();

	// todo: Delete below 4 function.
	bool isStateInitUpdateFinish();
	bool isStateCleanUpdateFinish();
	bool isStateGoHomePointUpdateFinish();
	bool isStateGoToChargerUpdateFinish();

};

class CleanModeFollowWall:public ACleanMode {
public:
	CleanModeFollowWall();

	~CleanModeFollowWall() override;

//	bool setNextAction() override;

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

	bool updateActionInStateClean()override ;

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

	bool updateActionInStateClean();
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
