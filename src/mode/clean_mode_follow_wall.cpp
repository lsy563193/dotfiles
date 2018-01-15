//
// Created by lsy563193 on 12/9/17.
//

#include <pp.h>
#include "robot.hpp"
#include "arch.hpp"
#include "dev.h"

Cells path_points;

CleanModeFollowWall::CleanModeFollowWall()
{
//	event_manager_register_handler(this);
//	event_manager_set_enable(true);
//	ROS_INFO("%s %d: Entering Follow wall mode\n=========================" , __FUNCTION__, __LINE__);
//	IMoveType::sp_mode_ = this;
	diff_timer_ = WALL_FOLLOW_TIME;
	speaker.play(VOICE_CLEANING_WALL_FOLLOW, false);
	clean_path_algorithm_.reset(new WFCleanPathAlgorithm);
	go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_));
}

CleanModeFollowWall::~CleanModeFollowWall()
{
/*
	IMoveType::sp_mode_ = nullptr;
	event_manager_set_enable(false);

	if (ev.key_clean_pressed)
	{
		speaker.play(VOICE_CLEANING_FINISHED);
		ROS_WARN("%s %d: Key clean pressed. Finish cleaning.", __FUNCTION__, __LINE__);
	}
	else if (ev.cliff_all_triggered)
	{
		speaker.play(VOICE_ERROR_LIFT_UP, false);
		speaker.play(VOICE_CLEANING_STOP);
		ROS_WARN("%s %d: Cliff all triggered. Finish cleaning.", __FUNCTION__, __LINE__);
	}
	else
	{
		speaker.play(VOICE_CLEANING_FINISHED);
		ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
	}
*/
}

bool CleanModeFollowWall::mapMark() {
	clean_path_algorithm_->displayCellPath(pointsGenerateCells(passed_path_));
	PP_WARN();
	if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{
		clean_map_.setCleaned(pointsGenerateCells(passed_path_));
		clean_map_.setBlocks();
		ROS_ERROR("-------------------------------------------------------");
		auto start = *passed_path_.begin();
		passed_path_.erase(std::remove_if(passed_path_.begin(),passed_path_.end(),[&start](Point32_t& it){
			return it.toCell() == start.toCell();
		}),passed_path_.end());
		clean_path_algorithm_->displayCellPath(pointsGenerateCells(passed_path_));
		ROS_ERROR("-------------------------------------------------------");
		clean_map_.setFollowWall(action_i_ == ac_follow_wall_left, passed_path_);
	}
	clean_map_.markRobot(CLEAN_MAP);
	clean_map_.print(CLEAN_MAP, getPosition().toCell().x, getPosition().toCell().y);
	passed_path_.clear();
	return false;
}

void CleanModeFollowWall::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);
	wheel.stop();

	// Wait for key released.
	bool long_press = false;
	while (key.getPressStatus())
	{
		if (!long_press && key.getPressTime() > 3)
		{
			ROS_WARN("%s %d: key clean long pressed.", __FUNCTION__, __LINE__);
			beeper.play_for_command(VALID);
			long_press = true;
		}
		usleep(20000);
	}

	if (long_press)
		ev.key_long_pressed = true;
	else
		ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}
//
//void CleanModeFollowWall::overCurrentWheelLeft(bool state_now, bool state_last)
//{
//	ROS_WARN("%s %d: Left wheel oc.", __FUNCTION__, __LINE__);
//	ev.oc_wheel_left = true;
//}
//
//void CleanModeFollowWall::overCurrentWheelRight(bool state_now, bool state_last)
//{
//	ROS_WARN("%s %d: Right wheel oc.", __FUNCTION__, __LINE__);
//	ev.oc_wheel_right = true;
//}
//
void CleanModeFollowWall::remoteMax(bool state_now, bool state_last)
{

	beeper.play_for_command(VALID);
	vacuum.switchToNext();
	remote.reset();
}
void CleanModeFollowWall::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);
	wheel.stop();
	ev.key_clean_pressed = true;
	remote.reset();
}

//void CleanModeFollowWall::remoteHome(bool state_now, bool state_last)
//{
//	if (sp_state == state_clean || action_i_ == ac_pause)
//	{
//		ROS_WARN("%s %d: remote home.", __FUNCTION__, __LINE__);
//		beeper.play_for_command(VALID);
//		ev.remote_home = true;
//	}
//	else
//	{
//		ROS_WARN("%s %d: remote home but not valid.", __FUNCTION__, __LINE__);
//		beeper.play_for_command(INVALID);
//	}
//	remote.reset();
//}
//
//void CleanModeFollowWall::remoteDirectionLeft(bool state_now, bool state_last)
//{
//	//todo: Just for debug
//	if (sp_state == state_clean)
//	{
//		beeper.play_for_command(VALID);
//		continue_point_ = getPosition();
//		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m, continue cell(%d, %d)", __FUNCTION__, __LINE__,
//				 battery.getVoltage(), continue_point_.x, continue_point_.y);
//		ev.battery_home = true;
//		go_home_for_low_battery_ = true;
//	}
//	else
//		beeper.play_for_command(INVALID);
//
//	remote.reset();
//}
//
//void CleanModeFollowWall::cliffAll(bool state_now, bool state_last)
//{
//	if (!ev.cliff_all_triggered)
//	{
//		ROS_WARN("%s %d: Cliff all.", __FUNCTION__, __LINE__);
//		ev.cliff_all_triggered = true;
//	}
//}
//
//void CleanModeFollowWall::batteryHome(bool state_now, bool state_last)
//{
//	if (sp_state == state_clean)
//	{
//		continue_point_ = getPosition();
//		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m, continue cell(%d, %d)", __FUNCTION__, __LINE__,
//				 battery.getVoltage(), continue_point_.x, continue_point_.y);
//		ev.battery_home = true;
//		go_home_for_low_battery_ = true;
//	}
//}
//
//void CleanModeFollowWall::chargeDetect(bool state_now, bool state_last)
//{
//	if (!ev.charge_detect && charger.isDirected())
//	{
//		ROS_WARN("%s %d: Charge detect!.", __FUNCTION__, __LINE__);
//		ev.charge_detect = charger.getChargeStatus();
//	}
//
//}
// End event handlers.

bool CleanModeFollowWall::wf_is_isolate(GridMap& map)
{
	int16_t	val = 0;
	int16_t x_min_forward, x_max_forward, y_min, y_max;
	map.getMapRange(CLEAN_MAP, &x_min_forward, &x_max_forward, &y_min, &y_max);
	Cell_t out_cell {int16_t(x_max_forward + 1),int16_t(y_max + 1)};
	Cell_t zero_home = {0, 0};

	map.markRobot(CLEAN_MAP);//note: To clear the obstacle when check isolated, please don't remove it!
	auto curr = getPosition().toCell();
	map.print(CLEAN_MAP, curr.x, curr.y);
	ROS_WARN("%s %d: curr(%d,%d),out(%d,%d)", __FUNCTION__, __LINE__, curr.x, curr.y,out_cell.x, out_cell.y);

	if ( out_cell != zero_home){
			val = wf_path_find_shortest_path(map, curr.x, curr.y, out_cell.x, out_cell.y, 0);
			val = (val < 0 || val == SCHAR_MAX) ? 0 : 1;
	} else {
		if (!map.isBlockAccessible(0, 0)) {
			val = wf_path_find_shortest_path(map, curr.x, curr.y, 0, 0, 0);
			if (val < 0 || val == SCHAR_MAX) {
				/* Robot start position is blocked. */
				val = wf_path_find_shortest_path(map, curr.x, curr.y, 0, 0, 0);

				if (val < 0 || val == SCHAR_MAX) {
					val = 0;
				} else {
					val = 1;
				};
			} else {
				val = 1;
			}
		} else {
			val = wf_path_find_shortest_path(map, curr.x, curr.y, 0, 0, 0);
			if (val < 0 || val == SCHAR_MAX)
				val = 0;
			else
				val = 1;
		}
	}
	return val != 0;
}


/*
 * Give a target point, find the shorest path from the current robot position to the
 * target position.
 *
 * @param xID	Robot x Coordinate
 * @param yID	Robot y Coordinate
 * @param endx	Target x Coordinate
 * @param endy	Target y Coordinate
 * @param bound	Limit to the search range to (xID, yID) and (endx, endy)
 *
 * @return	-2: Robot is trapped
 * 		-1: Path to target is not found
 * 		1:  Path to target is found
 * 		(totalCost: from function path_find_shortest_path_ranged)
 *
 */
int16_t CleanModeFollowWall::wf_path_find_shortest_path(GridMap& map, int16_t xID, int16_t yID, int16_t endx, int16_t endy, uint8_t bound)
{
	int16_t val;
	int16_t x_min, x_max, y_min, y_max;

	if (bound == 1) {
		/* If bound is set, set the search range. */
		x_min = (xID > endx ? endx : xID) - 8;
		x_max = (xID > endx ? xID : endx) + 8;
		y_min = (yID > endy ? endy : yID) - 8;
		y_max = (yID > endy ? yID : endy) + 8;
		ROS_INFO("shortest path(%d): endx: %d\tendy: %d\tx: %d - %d\ty: %d - %d\n", __LINE__, endx, endy, x_min, x_max, y_min, y_max);
		val =  wf_path_find_shortest_path_ranged(map, xID, yID, endx, endy, bound, x_min, x_max, y_min, y_max,false);
	} else {
		/* If bound is not set, set the search range to the whole costmap. */
		map.getMapRange(CLEAN_MAP, &x_min, &x_max, &y_min, &y_max);
		x_min = x_min - 8;
		x_max = x_max + 8;
		y_min = y_min - 8;
		y_max = y_max + 8;

		val =  wf_path_find_shortest_path_ranged(map, xID, yID, endx, endy, bound, x_min, x_max, y_min, y_max,false);
		ROS_INFO("shortest path(%d): endx: %d\tendy: %d\tx: %d - %d\ty: %d - %d\t return: %d\n", __LINE__, endx, endy, x_min, x_max, y_min, y_max, val);
	}
	return val;
}

int16_t CleanModeFollowWall::wf_path_find_shortest_path_ranged(GridMap& map, int16_t curr_x, int16_t curr_y, int16_t end_x, int16_t end_y, uint8_t bound, int16_t x_min, int16_t x_max, int16_t y_min, int16_t y_max,bool used_unknown)
{
	uint16_t	next;
	int16_t	totalCost, costAtCell, targetCost, dest_dir;
	int16_t i, j, m, n, tracex, tracey, tracex_tmp, tracey_tmp, passValue, nextPassValue, passSet, offset;
	CellState cs;

	path_points.clear();

	/* Find the direction of target with reference to the current robot position. */
	if (curr_x == end_x) {
		dest_dir = (curr_y > end_y ? MAP_NEG_Y : MAP_POS_Y);
	} else if (curr_y == end_y) {
		dest_dir = (curr_x > end_x ? MAP_NEG_X : MAP_POS_X);
	} else if(abs(curr_x - end_x) > abs(curr_y - end_y)) {
		dest_dir = (curr_x > end_x ? MAP_NEG_X : MAP_POS_X);
	} else {
		dest_dir = (curr_y > end_y ? MAP_NEG_Y : MAP_POS_Y);
	}

	/* Reset the cells in the shorest path costmap. */
	for (i = x_min - 1; i <= x_max + 1; ++i) {
		for (j = y_min - 1; j <= y_max + 1; ++j) {
			map.setCell(COST_MAP, (int32_t) i, (int32_t) j, COST_NO);
		}
	}

	/* Marked the obstcals to the shorest path costmap. */
	for (i = x_min - 1; i <= x_max + 1; ++i) {
		for (j = y_min - 1; j <= y_max + 1; ++j) {
			cs = map.getCell(CLEAN_MAP, i, j);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				//for (m = ROBOT_RIGHT_OFFSET + 1; m <= ROBOT_LEFT_OFFSET - 1; m++) {
				for (m = ROBOT_RIGHT_OFFSET; m <= ROBOT_LEFT_OFFSET; m++) {
					for (n = ROBOT_RIGHT_OFFSET; n <= ROBOT_LEFT_OFFSET; n++) {
						map.setCell(COST_MAP, (int32_t) (i + m), (int32_t) (j + n), COST_HIGH);
					}
				}
			}
			else if(cs == UNCLEAN && used_unknown)
				map.setCell(COST_MAP, (int32_t) (i), (int32_t) (j), COST_HIGH);
		}
	}

	// Now it is always finding the path from robot to target, so comment below sentence.
	// If needs to find path from target to robot, please uncomment below sentence.
	//if (map.getCell(COST_MAP, end_x, end_y, true) == COST_HIGH) {
	//	map.set_cell(COST_MAP, end_x, end_y, COST_NO);
	//}

	/* Set the current robot position has the cost value of 1. */
	map.setCell(COST_MAP, (int32_t) curr_x, (int32_t) curr_y, COST_1);

	/*
	 * Find the path to target from the current robot position. Set the cell values
	 * in shorest path costmap either 1, 2, 3, 4 or 5. This is a method like A-Star, starting
	 * from a start point, init the cells one level away, until we reach the target.
	 */
	offset = 0;
	passSet = 1;
	passValue = 1;
	nextPassValue = 2;
	while (map.getCell(COST_MAP, end_x, end_y) == COST_NO && passSet == 1) {
		offset++;
		passSet = 0;

		/*
		 * The following 2 for loops is for optimise the computational time.
		 * Since there is not need to go through the whole costmap for seaching the
		 * cell that have the next pass value.
		 *
		 * It can use the offset to limit the range of searching, since in each loop
		 * the cell (x -/+ offset, y -/+ offset) would be set only. The cells far away
		 * to the robot position won't be set.
		 */
		for (i = curr_x - offset; i <= curr_x + offset; i++) {
			if (i < x_min || i > x_max)
				continue;

			for (j = curr_y - offset; j <= curr_y + offset; j++) {
				if (j < y_min || j > y_max)
					continue;

				/* Found a cell that has a pass value equal to the current pass value. */
				if(map.getCell(COST_MAP, i, j) == passValue) {
					/* Set the lower cell of the cell which has the pass value equal to current pass value. */
					if (map.getCell(COST_MAP, i - 1, j) == COST_NO) {
						map.setCell(COST_MAP, (int32_t) (i - 1), (int32_t) j, (CellState) nextPassValue);
						passSet = 1;
					}

					/* Set the upper cell of the cell which has the pass value equal to current pass value. */
					if (map.getCell(COST_MAP, i + 1, j) == COST_NO) {
						map.setCell(COST_MAP, (int32_t) (i + 1), (int32_t) j, (CellState) nextPassValue);
						passSet = 1;
					}

					/* Set the cell on the right hand side of the cell which has the pass value equal to current pass value. */
					if (map.getCell(COST_MAP, i, j - 1) == COST_NO) {
						map.setCell(COST_MAP, (int32_t) i, (int32_t) (j - 1), (CellState) nextPassValue);
						passSet = 1;
					}

					/* Set the cell on the left hand side of the cell which has the pass value equal to current pass value. */
					if (map.getCell(COST_MAP, i, j + 1) == COST_NO) {
						map.setCell(COST_MAP, (int32_t) i, (int32_t) (j + 1), (CellState) nextPassValue);
						passSet = 1;
					}
				}
			}
		}

		/* Update the pass value. */
		passValue = nextPassValue;
		nextPassValue++;

		/* Reset the pass value, pass value can only between 1 to 5. */
		if(nextPassValue == COST_PATH)
			nextPassValue = 1;
	}

	/* The target position still have a cost of 0, which mean it is not reachable. */
	totalCost = 0;
	if (map.getCell(COST_MAP, end_x, end_y) == COST_NO || map.getCell(COST_MAP, end_x, end_y) == COST_HIGH) {
		ROS_WARN("%s, %d: target point (%d, %d) is not reachable(%d), return -2.", __FUNCTION__, __LINE__, end_x, end_y,
				 map.getCell(COST_MAP, end_x, end_y));
#if	DEBUG_COST_MAP
		clean_map_.print(COST_MAP, end_x, end_y);
#endif
		return -2;
	}

	/* If bound is set, not path tracing is needed. */
	if (bound == 1) {
		return 1;
	}

	/*
	 * Start from the target position, trace back the path by the cost level.
	 * Value of cells on the path is set to 6. Stops when reach the current
	 * robot position.
	 *
	 * The last robot direction is use, this is to avoid using the path that
	 * have the same direction as previous action.
	 */
	Cell_t t;
	t.x = tracex = tracex_tmp = end_x;
	t.y = tracey = tracey_tmp = end_y;
	path_points.push_back(t);

	next = 0;
	dest_dir = (new_dir_ == MAP_POS_Y || new_dir_ == MAP_NEG_Y) ? 1: 0;
	ROS_INFO("%s %d: dest dir: %d", __FUNCTION__, __LINE__, dest_dir);
	while (tracex != curr_x || tracey != curr_y) {
		costAtCell = map.getCell(COST_MAP, tracex, tracey);
		targetCost = costAtCell - 1;

		/* Reset target cost to 5, since cost only set from 1 to 5 in the shorest path costmap. */
		if (targetCost == 0)
			targetCost = COST_5;

		/* Set the cell value to 6 if the cells is on the path. */
		map.setCell(COST_MAP, (int32_t) tracex, (int32_t) tracey, COST_PATH);

#define COST_SOUTH	{											\
				if (next == 0 && (map.getCell(COST_MAP, tracex - 1, tracey) == targetCost)) {	\
					tracex--;								\
					next = 1;								\
					dest_dir = 1;								\
				}										\
			}

#define COST_WEST	{											\
				if (next == 0 && (map.getCell(COST_MAP, tracex, tracey - 1) == targetCost)) {	\
					tracey--;								\
					next = 1;								\
					dest_dir = 0;								\
				}										\
			}

#define COST_EAST	{											\
				if (next == 0 && (map.getCell(COST_MAP, tracex, tracey + 1) == targetCost)) {	\
					tracey++;								\
					next = 1;								\
					dest_dir = 0;								\
				}										\
			}

#define COST_NORTH	{											\
				if (next == 0 && map.getCell(COST_MAP, tracex + 1, tracey) == targetCost) {	\
					tracex++;								\
					next = 1;								\
					dest_dir = 1;								\
				}										\
			}

		next = 0;
		if (dest_dir == 0) {
			COST_WEST
			COST_EAST
			COST_SOUTH
			COST_NORTH
		} else {
			COST_SOUTH
			COST_NORTH
			COST_WEST
			COST_EAST
		}

#undef COST_EAST
#undef COST_SOUTH
#undef COST_WEST
#undef COST_NORTH

		totalCost++;
		if (path_points.back().x != tracex && path_points.back().y != tracey) {
			t.x = tracex_tmp;
			t.y = tracey_tmp;
			path_points.push_back(t);
		}
		tracex_tmp = tracex;
		tracey_tmp = tracey;
	}
	map.setCell(COST_MAP, (int32_t) tracex, (int32_t) tracey, COST_PATH);

	t.x = tracex_tmp;
	t.y = tracey_tmp;
	path_points.push_back(t);

//	path_display_path_points(path_points);

	return totalCost;
}

bool CleanModeFollowWall::updateActionInStateClean()
{
	ROS_INFO_FL();
	sp_action_.reset();// to mark in destructor
	old_dir_ = new_dir_;
	if (reach_cleaned_count_ == 0) {
		if (generatePath(clean_map_, getPosition(), old_dir_, plan_path_)) {
			new_dir_ = plan_path_.front().th;
			plan_path_.pop_front();
			robot::instance()->pubCleanMapMarkers(clean_map_, pointsGenerateCells(plan_path_));
		}
	}
	else if (reach_cleaned_count_ <= 3) {

		if (wf_is_isolate(clean_map_)) {
			if (generatePath(clean_map_, getPosition(), old_dir_, plan_path_)) {
				new_dir_ = plan_path_.front().th;
				plan_path_.pop_front();
				robot::instance()->pubCleanMapMarkers(clean_map_, pointsGenerateCells(plan_path_));
			}
		}
		else {
			ROS_WARN("%s,%d:follow clean finish", __func__, __LINE__);
/*			ROS_WARN("%s,%d:follow clean finish,did not find charge", __func__, __LINE__);
			sp_state = state_go_home_point;
			go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_));
			sp_state->init();
			action_i_ = ac_null;*/
			return false;
		}
	}else{
		return false;
	}


	PP_INFO();
	if (plan_path_.empty()) {
		ROS_WARN("%s,%d: mt_follow_wall_left", __FUNCTION__, __LINE__);
		action_i_ = ac_follow_wall_left;
		genNextAction();
		ROS_WARN("%s,%d: mt_follow_wall_left", __FUNCTION__, __LINE__);
	}
	else {
		action_i_ = ac_linear;
		genNextAction();
		ROS_WARN("%s,%d: ac_linear", __FUNCTION__, __LINE__);
	}
	return true;
}

bool CleanModeFollowWall::MoveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt) {
//	ROS_INFO("reach_cleaned_count_ = %d, reach_cleaned_count_save = %d", reach_cleaned_count_, reach_cleaned_count_save);
	if(reach_cleaned_count_ > reach_cleaned_count_save)
	{
		reach_cleaned_count_save = reach_cleaned_count_;
		return true;
	}
	return false;
}

void CleanModeFollowWall::switchInStateClean() {
	sp_state = state_go_home_point;
	ROS_INFO("%s %d: home_cells_.size(%lu)", __FUNCTION__, __LINE__, home_points_.size());
	go_home_path_algorithm_.reset();
	go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_));
	sp_state->init();
	action_i_ = ac_null;
	genNextAction();
}

bool CleanModeFollowWall::generatePath(GridMap &map, const Point32_t &curr, const int &last_dir, Points &targets)
{
	if (targets.empty()) {//fw ->linear
		auto curr = getPosition();
		fw_map.reset(CLEAN_MAP);
		auto angle = (reach_cleaned_count_ != 0 && reach_cleaned_count_ <= 3) ? -900 : 0;
		auto point = getPosition();
		point.th = ranged_angle(curr.th + angle);
		targets.push_back(point);
		ROS_WARN("curr.th = %d, angle = %d,point.th(%d)", curr.th, angle,point.th);
		point = point.getRelative(8 * 1000, 0);
		targets.push_back(point);
		ROS_WARN("%s,%d: empty! point(%d, %d, %d)", __FUNCTION__, __LINE__,targets.back().x, targets.back().y, targets.back().th);
	}
	else//linear->fw
	{
		targets.clear();
		targets.push_back(getPosition());
		ROS_WARN("%s,%d: not empty! point(%d, %d, %d)", __FUNCTION__, __LINE__,targets.back().x, targets.back().y, targets.back().th);
	}
	return true;
}
