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

	void setNextMode(int next_mode);

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
		ac_turn,//6
		ac_forward,
		ac_follow_wall_left,//8
		ac_follow_wall_right,
		ac_back,//10
//		ac_movement_follow_wall_left,
//		ac_movement_follow_wall_right,
		ac_movement_go_charger,
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

class ACleanMode:public Mode,public PathAlgorithm
{
public:
	ACleanMode();
	bool isFinish();
	virtual bool setNextState();
	virtual bool setNextMoveType();
	virtual bool setNextAction();
	void genMoveAction();
	void resetTriggeredValue();

	virtual bool map_mark() = 0;

	virtual bool MovementFollowWallisFinish();
	Cell_t updatePath();

protected:

	uint8_t saveFollowWall(bool is_left);

	bool st_is_finish();
	bool mt_is_turn();
	bool mt_is_linear();
	bool mt_is_follow_wall();
	bool mt_is_go_charger();
	bool mt_is_null();
	bool action_is_movement();
	bool ac_is_forward();
	bool ac_is_follow_wall();
	bool ac_is_turn();
	bool ac_is_back();
	void st_init(int);
	void mt_init(int);
	std::vector<Cell_t> temp_fw_cells;
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
  int16_t turn_target_angle_{};
	int state_i_{st_null};
	enum {
		st_null,
		st_clean,
		st_go_home_point,
		st_go_charger,
		st_trapped,
		st_tmp_spot,
		st_self_check,
		st_exploration,
	};
	int move_type_i_{mt_null};
	enum {
		mt_null,
		mt_linear,
		mt_follow_wall_left,
		mt_follow_wall_right,
		mt_go_charger,
	};
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
		bool isInitState();
	void register_events(void);

};

class CleanModeNav:public ACleanMode
{
public:
	CleanModeNav();
	~CleanModeNav() override ;

	uint8_t setFollowWall();
	bool setNextMoveType() override ;
	bool map_mark() override ;
//	bool isFinish();
	bool isExit();

	void key_clean(bool state_now, bool state_last) override ;
	void cliff_all(bool state_now, bool state_last) override ;

private:
	bool MovementFollowWallisFinish() override ;
	bool isNewLineReach();
	bool isOverOriginLine();

// For path planning.

protected:
//	Path_t home_point_{};
public:
	/*
	 * @author Patrick Chow / Lin Shao Yue
	 * @last modify by Austin Liu
	 *
	 * This function is for finding path to unclean area.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 * @param: MapDirection last_dir, the direction of last movement.
	 *
	 * @return: Path_t path, the path to unclean area.
	 */
	Path_t generatePath(GridMap &map, const Cell_t &curr_cell, const MapDirection &last_dir) override;

private:
	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for finding path to unclean area in the same lane.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 *
	 * @return: Path_t path, the path to unclean area in the same lane.
	 */
	Path_t findTargetInSameLane(GridMap &map, const Cell_t &curr_cell);

	/*
	 * @author Lin Shao Yue
	 * @last modify by Austin Liu
	 *
	 * This function is for finding all possiable targets in the map, judging by the boundary between
	 * cleaned and reachable unclean area.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 * @param: BoundingBox2 b_map, generate by map, for simplifying code.
	 *
	 * @return: TargetList, a deque of possible targets.
	 */
	TargetList filterAllPossibleTargets(GridMap &map, const Cell_t &curr_cell, BoundingBox2 &b_map);

	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for filtering targets with their cost in the COST_MAP.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 * @param: TargetList possible_targets, input target list.
	 *
	 * @return: TargetList, a deque of reachable targets.
	 */
	TargetList getReachableTargets(GridMap &map, const Cell_t &curr_cell, TargetList &possible_targets);

	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for tracing the path from start cell to targets.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: TargetList target_list, input target list.
	 * @param: Cell_t start, the start cell.
	 *
	 * @return: PathList, a deque of paths from start cell to the input targets.
	 */
	PathList tracePathsToTargets(GridMap &map, const TargetList &target_list, const Cell_t& start);

	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for selecting the best target in the input paths according to their path track.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: PathLIst paths, the input paths.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 *
	 * @return: true if best target is selected.
	 *          false if there is no target that match the condictions.
	 *          Cell_t best_target, the selected best target.
	 */
	bool filterPathsToSelectTarget(GridMap &map, const PathList &paths, const Cell_t &curr_cell, Cell_t &best_target);

};

class CleanModeFollowWall:public ACleanMode
{
public:
	CleanModeFollowWall();
	~CleanModeFollowWall() override ;

	bool setNextMoveType() override ;
	bool map_mark() override;

	Path_t generatePath(GridMap &map, const Cell_t &curr_cell, const MapDirection &last_dir);

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

	bool setNextMoveType() override ;
	bool map_mark() override;
	Path_t generatePath(GridMap &map, const Cell_t &curr_cell, const MapDirection &last_dir) override;

private:
protected:
//	Path_t home_point_{};
private:

};
#endif //PP_MODE_H_H
