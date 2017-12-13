//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_MODE_H_H
#define PP_MODE_H_H

#include "move_type_plan.hpp"
#include "path_algorithm.h"
#include "event_manager.h"
#include "boost/shared_ptr.hpp"
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

protected:
	static boost::shared_ptr<IAction> sp_action_;
	int mode_i_{ac_null};

	int action_i_{ac_null};
	enum {
		ac_null,
		ac_open_gyro,
		ac_back_form_charger,//2
		ac_open_lidar,
		ac_align,//4
		ac_open_slam,
		ac_linear,
		ac_turn,//6
		ac_forward,
		ac_follow_wall_left,//8
		ac_follow_wall_right,
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
	};

private:

};

class ModeIdle:public Mode
{
public:
	ModeIdle();
	~ModeIdle();
	bool isExit();
	void remote_cleaning(bool state_now, bool state_last);
	void remote_direction_left(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_direction_right(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_direction_forward(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_home(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_spot(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_wall_follow(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_clean(bool state_now, bool state_last){remote_cleaning(state_now,state_last);}
	void remote_max(bool state_now, bool state_last) override ;
	void remote_plan(bool state_now, bool state_last) override ;
	void key_clean(bool state_now, bool state_last) override;
	void charge_detect(bool state_now, bool state_last) override ;

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
	void remote_clean(bool state_now, bool state_last);
	void key_clean(bool state_now, bool state_last);
	void charge_detect(bool state_now, bool state_last);
	void rcon(bool state_now, bool state_last);
	void remote_plan(bool state_now, bool state_last);

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
	void remote_clean(bool state_now, bool state_last) override ;
	void key_clean(bool state_now, bool state_last) override ;
	void remote_plan(bool state_now, bool state_last) override ;

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
	void remote_clean(bool state_now, bool state_last) override ;
	void remote_direction_left(bool state_now, bool state_last) override ;
	void remote_direction_right(bool state_now, bool state_last) override ;
	void remote_direction_forward(bool state_now, bool state_last) override ;
	void key_clean(bool state_now, bool state_last) override ;

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

	virtual bool map_mark() = 0;

	virtual bool MovementFollowWallisFinish();
	Cell_t updatePath();

protected:

	APathAlgorithm* clean_path_algorithm_;
	APathAlgorithm* go_home_path_algorithm_;
	uint8_t saveFollowWall(bool is_left);
	bool isInitState();

	bool st_is_finish();
	bool action_is_movement();
	bool ac_is_forward();
	bool ac_is_follow_wall();
	bool ac_is_turn();
	bool ac_is_back();
	bool ac_is_go_to_charger();
	void st_init(int);
	std::vector<Cell_t> temp_fw_cells;
	TargetList home_cells_;
	static Path_t passed_path_;
	static Path_t plan_path_;
	Point32_t cm_target_p_;
	Point32_t cm_origin_p_;
	static Cell_t last_;
	uint32_t start_timer_;
	uint32_t diff_timer_;

	const int ISOLATE_COUNT_LIMIT = 4;
	MapDirection old_dir_{MAP_POS_X};
	MapDirection new_dir_{MAP_POS_X};
//	static boost::shared_ptr<State> sp_state_;
//	static boost::shared_ptr<IMoveType> sp_move_type_;
	int state_i_{st_null};
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
//	int move_type_i_{mt_null};
//	enum {
//		mt_null,
//		mt_linear,
//		mt_follow_wall_left,
//		mt_follow_wall_right,
//		mt_go_to_charger,
//	};
//	int movement_i_ {mv_null};
	enum {
//		mv_null,
		mv_back,
		mv_turn,
		mv_forward,
		mv_turn2,
		mv_follow_wall,
		mv_go_charger,
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
	bool map_mark() override ;
	bool isFinish() override ;
	bool isExit();

	bool setNextAction();
	void key_clean(bool state_now, bool state_last) override ;
	void remote_clean(bool state_now, bool state_last) override ;
	void remote_home(bool state_now, bool state_last) override ;
	void cliff_all(bool state_now, bool state_last) override ;

private:
	bool MovementFollowWallisFinish() override ;
	bool isNewLineReach();
	bool isOverOriginLine();

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

	bool map_mark() override;


	int16_t wf_path_find_shortest_path(int16_t xID, int16_t yID, int16_t endx, int16_t endy, uint8_t bound);
	int16_t wf_path_find_shortest_path_ranged(int16_t curr_x, int16_t curr_y, int16_t end_x, int16_t end_y, uint8_t bound, int16_t x_min, int16_t x_max, int16_t y_min, int16_t y_max);
	bool wf_is_isolate();
private:
protected:
//	Path_t home_point_{};
private:

};

class CleanModeSpot:public ACleanMode
{
public:
	CleanModeSpot();
	~CleanModeSpot() override ;

	bool map_mark() override;

private:
protected:
//	Path_t home_point_{};
private:

};
#endif //PP_MODE_H_H
