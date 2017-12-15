//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_MODE_H_H
#define PP_MODE_H_H

#include "path_algorithm.h"
#include "event_manager.h"
#include "boost/shared_ptr.hpp"
#include "move_type.hpp"

class Mode:public EventHandle {
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
		cm_exploration
	};

	int next_mode_i_;

	int action_i_{ac_null};
	enum {
		ac_null,
		ac_open_gyro,
		ac_back_form_charger,//2
		ac_open_lidar,
		ac_align,//4
		ac_open_slam,
		ac_linear,//6
		ac_follow_wall_left,
		ac_follow_wall_right,//8
		ac_turn,
		ac_forward,
		ac_back,//10
//		ac_movement_follow_wall_left,
//		ac_movement_follow_wall_right,
		ac_go_to_charger,
		ac_idle,
		ac_sleep,
		ac_charge,
		ac_turn_for_charger,
		ac_movement_stay,
		ac_movement_direct_go,
		ac_pause,
		ac_self_check
	};

protected:
	static boost::shared_ptr<IAction> sp_action_;
	int mode_i_{ac_null};

private:

};

class ModeIdle:public Mode
{
public:
	ModeIdle();
	~ModeIdle() override;
	bool isExit() override;
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

protected:
	std::vector<Cell_t> temp_fw_cells;
private:
	void register_events(void);

	bool plan_activated_status_;
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

	IAction* getNextAction();

	// For exit event handling.
	void remoteClean(bool state_now, bool state_last) override ;
	void keyClean(bool state_now, bool state_last) override ;
	void remotePlan(bool state_now, bool state_last) override ;

private:
	bool plan_activated_status_;
	bool directly_charge_;
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
	bool time_out_;

};

class ACleanMode:public Mode
{
public:
	ACleanMode();
	bool isFinish();
	void setNextMode(int next);
	virtual bool setNextState();
	virtual bool setNextAction();
	void genNextAction();
	void resetTriggeredValue();

	virtual bool mapMark() = 0;

	virtual bool MovementFollowWallisFinish();
	Cell_t updatePath();

	static Path_t passed_path_;
	static Path_t plan_path_;

	MapDirection old_dir_{MAP_POS_X};
	MapDirection new_dir_{MAP_POS_X};
protected:

	boost::shared_ptr<APathAlgorithm> clean_path_algorithm_;
	boost::shared_ptr<APathAlgorithm> go_home_path_algorithm_;
	uint8_t saveFollowWall(bool is_left);
	virtual bool isInitState();
	void stateInit(int);
	std::vector<Cell_t> temp_fw_cells;
	TargetList home_cells_;
	static Cell_t last_;

	int state_i_{st_clean};
	enum {
		st_null,
		st_clean,
		st_go_home_point,
		st_go_to_charger,
		st_trapped,
		st_tmp_spot,
		st_self_check,
		st_exploration,
	};

private:
	void register_events(void);

};

class CleanModeNav:public ACleanMode
{
public:
	CleanModeNav();
	~CleanModeNav() override ;

	uint8_t setFollowWall();
	bool mapMark() override ;
	bool isFinish() override ;
	bool isExit();

	bool setNextAction();
	void keyClean(bool state_now, bool state_last) override ;
	void remoteClean(bool state_now, bool state_last) override ;
	void remoteHome(bool state_now, bool state_last) override ;
	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;
//	void overCurrentBrushLeft(bool state_now, bool state_last);
//	void overCurrentBrushMain(bool state_now, bool state_last);
//	void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
//	void overCurrentSuction(bool state_now, bool state_last);

private:
	bool MovementFollowWallisFinish() override ;
	bool isNewLineReach();
	bool isOverOriginLine();
	bool enterPause();
	bool resumePause();
	bool switchToGoHomePointState();

	bool paused_;
	bool has_aligned_and_open_slam;
	float paused_odom_angle_;
	bool moved_during_pause_;

// For path planning.

protected:
//	Path_t home_point_{};
public:

};

class CleanModeFollowWall:public ACleanMode
{
public:
	CleanModeFollowWall();
	~CleanModeFollowWall() override ;

	bool mapMark() override;


	int16_t wf_path_find_shortest_path(int16_t xID, int16_t yID, int16_t endx, int16_t endy, uint8_t bound);
	int16_t wf_path_find_shortest_path_ranged(int16_t curr_x, int16_t curr_y, int16_t end_x, int16_t end_y, uint8_t bound, int16_t x_min, int16_t x_max, int16_t y_min, int16_t y_max);
	bool wf_is_isolate();
private:
	uint32_t diff_timer_;
protected:
//	Path_t home_point_{};
private:

};

class CleanModeSpot:public ACleanMode
{
public:
	CleanModeSpot();
	~CleanModeSpot() = default;

	bool mapMark() override;

private:
protected:
//	Path_t home_point_{};
private:

};
#endif //PP_MODE_H_H
