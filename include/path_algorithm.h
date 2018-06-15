//
// Created by austin on 17-12-3.
//

#ifndef PP_PATH_ALGORITHM_H
#define PP_PATH_ALGORITHM_H

#include <mathematics.h>
#include <deque>
#include "BoundingBox.h"

using func_compare_t =  std::function<bool(const Cell_t &next)>;
using func_compare_two_t =  std::function<bool(const Cell_t &neighbor,const Cell_t& next)>;
extern const Cell_t cell_direction_[9];
extern const Cell_t cell_direction_4[4];

typedef std::deque<Cells> PathList;

class GridMap;

class IsTarget
{
public:
	IsTarget(GridMap* p_map,const BoundingBox2& bound):p_map_(p_map),target_bound_(bound) { };
    bool operator()(const Cell_t &c_it) ;

private:
	GridMap* p_map_;
	BoundingBox2 target_bound_;
};

class TargetVal {
public:
	TargetVal(GridMap* p_map,CellState val):p_map_(p_map),val_(val) {};

	bool operator()(const Cell_t& c_it);
private:
	GridMap* p_map_;
	CellState val_;
};

class isAccessable {
public:
	isAccessable(GridMap *p_map,func_compare_two_t external_condition = nullptr,
				 const BoundingBox2& bound_ = BoundingBox2{});


	bool operator()(const Cell_t &next, const Cell_t &neighbor) ;

private:
	BoundingBox2 bound_{};
	GridMap *p_map_{};
	func_compare_two_t external_condition_{};
};

class APathAlgorithm
{
public:
	virtual bool generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path) = 0;

	virtual bool checkTrapped(GridMap &map, const Cell_t &curr_cell) {return true;};

	bool shift_path(GridMap &map, const Cell_t &p1, Cell_t &p2, Cell_t &p3, int num,bool is_first, bool is_reveave, const func_compare_two_t& expand_condition);

	virtual void optimizePath(GridMap &map, Cells &path, Dir_t& pri_dir, const func_compare_two_t& expand_condition);
	void findPath(GridMap &map, const Cell_t &start, const Cell_t &target, Cells &path, Dir_t last_i);

	void flood_fill(const Cell_t& curr);
	bool dijkstra(GridMap &map, const Cell_t &curr_cell, Cells &targets, bool is_stop, func_compare_t is_target,
				  func_compare_two_t isAccessible);
	uint16_t dijkstraCountCleanedArea(GridMap& map, Point_t curr, Cells &targets);
	public:
	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for checking the shortest and most reasonable path from start cell to target cell.
	 * The target of this function MUST be reachable. But we will use reverse algorithm to find the path,
	 * because it will have a better trace back process, the path will be more reasonable than positive
	 * algorithm.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.(COST_MAP data will be rewritten.)
	 * @param: Cell_t start, the start cell.
	 * @param: Cell_t target, the target cell.
	 * @param: MapDirection last_dir, the direction of robot last moving.
	 * @param: Boundery between min_corner and max_corner
	 * @param: min corner
	 * @param: max corner
	 *
	 * @return: Cells path, the shortest path from start cell to target cell.
	 */
//	Cells findShortestPath(GridMap &map, const Cell_t &start,
//							  const Cell_t &target, const Dir_t &last_dir, bool use_unknown,bool bound,Cell_t min_corner ,Cell_t max_corner);

	bool isAccessible(const Cell_t &c_it, const BoundingBox2& bound, GridMap& map);

	using cmp_condition_t = std::function<bool(const Cell_t&)>;

	std::unique_ptr<Cells> shortestPath(const Cell_t &start, const Cell_t& goal, const cmp_condition_t in_search_range, Dir_t dir);
//	bool generateShortestPath(GridMap &map, const Point_t &curr,const Point_t &target, const Dir_t &last_dir, Points &plan_path);

	/*
	 * @
	 *
	 */
	bool isTargetReachable(GridMap map,Cell_t target);

	using cmp_two = std::function<bool(const Cell_t& l, const Cell_t& r)>;
	using cmp_one = std::function<bool(const Cell_t& cell)>;
	std::unique_ptr<Cell_t> find_target(const Cell_t &start, cmp_one is_target, cmp_one is_accessible,
														const cmp_two &cmp_lambda) ;
protected:

	/*
	 * @author Lin Shao Yue
	 * @last modify by Austin Liu
	 *
	 * This function is for checking if robot is trapped using Dijkstra algorithm and comparing the clean
	 * area proportion.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 *
	 * @return: true, robot is trapped.
	 *          false, robot is not trapped.
	 */
	bool checkTrappedUsingDijkstra(GridMap &map, const Cell_t &curr_cell);
};

//typedef BoundingBox2(*RangeFunction)(const Cell_t&, const BoundingBox2& bound);
using RangeFunction = std::function<BoundingBox2()>;

class BestTargetFilter {
public:
	BestTargetFilter(std::string name,RangeFunction target_bound_func,RangeFunction range_bound_func,  bool is_toward_pos,bool is_forbit_turn=false):
			name_(name), update_target_bound(target_bound_func),update_range_bound(range_bound_func),  is_toward_pos_(is_toward_pos), is_forbit_turn_(is_forbit_turn){};

	void displayName(){
		std::cout << name_ << std::endl;
	};
	void updateTargetAndRangeBound(){
		target_bound = update_target_bound();
		range_bound = update_range_bound();
	};

	bool towardPos(){
		return is_toward_pos_;
	};
//private:
//	bool is_toward_pos_=false;
	BoundingBox2 target_bound;
	BoundingBox2 range_bound;
	std::string name_;
	bool is_toward_pos_{};
	bool is_forbit_turn_{};
	RangeFunction update_target_bound;
	RangeFunction update_range_bound;
};

class NavCleanPathAlgorithm: public APathAlgorithm
{
	/*
	 * @author Patrick Chow / Lin Shao Yue
	 * @last modify by Austin Liu
	 *
	 * This function is for finding path to unclean area.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Point_t curr, the current cell of robot.
	 * @param: int last_dir, the direction of last movement.
	 *
	 * @return: Points path, the path to unclean area.
	 * @return: bool, true if generating succeeds.
	 */
public:
	bool generatePath(GridMap &map, const Point_t &curr_p, const Dir_t &last_dir, Points &plan_path) override;
	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for finding path to unclean area in the same lane.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 *
	 * @return: Cells path, the path to unclean area in the same lane.
	 */
    bool should_follow_wall(){
		return ( curr_filter_ == &filter_after_obstacle_pos
				||	curr_filter_ == &filter_after_obstacle_neg
				||	curr_filter_ == &filter_top_of_y_axis_neg
				||	curr_filter_ == &filter_top_of_y_axis_pos
				||	curr_filter_ == &filter_next_line_neg
				||	curr_filter_ == &filter_next_line_pos
		)
				;
	};
    bool is_pox_y(){
		return curr_filter_ == &filter_top_of_y_axis_neg || curr_filter_ == &filter_top_of_y_axis_pos ? !curr_filter_->towardPos()
																									  : curr_filter_->towardPos();
	};

private:
	using pair_bb = std::tuple<BoundingBox2, BoundingBox2,Dir_t>;
	std::unique_ptr<std::deque<BestTargetFilter*>> generateBounds(GridMap& map);
	void optimizePath(GridMap &map, Cells &path, Dir_t& pri_dir,const func_compare_two_t& expand_condition) override ;

	bool checkTrapped(GridMap &map, const Cell_t &curr_cell) override ;
#if !USE_NEW_PATH_PLAN
//	std::unique_ptr<Cells> findTargetInSameLane(GridMap &map, const Cell_t &curr_cell);
/////////////////////////////////////////////////////////////////
	RangeFunction range_curr_line = [&](){
		return BoundingBox2{curr_filter_->target_bound.min, curr_filter_->target_bound.max};
	};

	RangeFunction range_next_line = [&](){
		auto d_min_y = (curr_filter_->is_toward_pos_) ? correct_curr_.y: curr_filter_->target_bound.min.y;
		auto d_max_y = (curr_filter_->is_toward_pos_) ? curr_filter_->target_bound.max.y : correct_curr_.y;
		return BoundingBox2{Cell_t{curr_filter_->target_bound.min.x,d_min_y} , Cell_t{curr_filter_->target_bound.max.x,d_max_y}};
	};

	RangeFunction range_after_obstacle = [&](){
		int16_t d_min_x = isPos(priority_dir) ? correct_curr_.x : curr_filter_->target_bound.min.x;
		int16_t d_max_x = isPos(priority_dir) ? curr_filter_->target_bound.max.x : correct_curr_.x;
		int16_t d_min_y = curr_filter_->is_toward_pos_ ? correct_curr_.y : correct_curr_.y-2;
		int16_t d_max_y = curr_filter_->is_toward_pos_ ? correct_curr_.y+2: correct_curr_.y;
		return BoundingBox2{Cell_t{d_min_x,d_min_y}, Cell_t{d_max_x, d_max_y}};
	};
	RangeFunction range_top_of_y_axis = [&](){
		int16_t d_min_x = isPos(priority_dir) ? correct_curr_.x : curr_filter_->target_bound.min.x;
		int16_t d_max_x = isPos(priority_dir) ? curr_filter_->target_bound.max.x : correct_curr_.x;
		int16_t d_min_y = curr_filter_->is_toward_pos_ ? correct_curr_.y-1 : correct_curr_.y-2;
		int16_t d_max_y = curr_filter_->is_toward_pos_ ? correct_curr_.y+2 : correct_curr_.y+1;
		return BoundingBox2{Cell_t{d_min_x,d_min_y}, Cell_t{d_max_x, d_max_y}};
	};
	RangeFunction range_p3p = [&](){
		int16_t dy = curr_filter_->towardPos() ? correct_curr_.y: int16_t(correct_curr_.y -1);
		return BoundingBox2{Cell_t{map_bound.min.x, dy}, map_bound.max};
	};
////////////////////////////////////////////////////////////

	RangeFunction target_curr_line = [&](){
		Cell_t min = curr_filter_->towardPos() ? correct_curr_ : Cell_t{map_bound.min.x, correct_curr_.y};
		Cell_t max = curr_filter_->towardPos() ? Cell_t{map_bound.max.x, correct_curr_.y} : correct_curr_;
		return BoundingBox2{min, max};
	};

	RangeFunction target_after_obstacle = [&](){
		int16_t dx1 = isPos(priority_dir) ? 3 : -5;
		int16_t dx2 = isPos(priority_dir) ? 5 : -3;
		return BoundingBox2{correct_curr_ + Cell_t{dx1,0}, correct_curr_ + Cell_t{dx2,0}};
	};

	RangeFunction target_next_line = [&](){
		int16_t dy = correct_curr_.y + (curr_filter_->towardPos() ? 2:-2);
		return BoundingBox2{Cell_t{curr_bound.min.x, dy}, Cell_t{curr_bound.max.x, dy}};
	};

	RangeFunction target_p3p = [&](){
		return BoundingBox2{Cell_t{map_bound.min.x, (int16_t)(correct_curr_.y + 1)}, map_bound.max};
	};

	RangeFunction target_top_of_y_axis = [&](){
		int16_t dy = curr_filter_->towardPos()? 2 : -2;
		int16_t dx1 = (isPos(priority_dir)) ? 3 : -5;
		int16_t dx2 = (isPos(priority_dir)) ? 5 : -3;
		return BoundingBox2{correct_curr_ + Cell_t{dx1, dy}, correct_curr_+ Cell_t{dx2, dy}};
	};

	RangeFunction target_all =[&](){
		return BoundingBox2{map_bound.min, map_bound.max};
	};

public:
	BestTargetFilter filter_curr_line_pos{"filter_curr_line_pos", target_curr_line, range_curr_line, true};

	BestTargetFilter filter_curr_line_neg{"filter_curr_line_neg", target_curr_line, range_curr_line, false};

	BestTargetFilter filter_after_obstacle_neg{"filter_after_obstacle_neg", target_after_obstacle, range_after_obstacle, false};

	BestTargetFilter filter_after_obstacle_pos{"filter_after_obstacle_pos", target_after_obstacle, range_after_obstacle, true};

	BestTargetFilter filter_next_line_pos{"filter_next_line_pos", target_next_line, range_next_line, true};

	BestTargetFilter filter_next_line_neg{"filter_next_line_neg", target_next_line , range_next_line, false};

	BestTargetFilter filter_pos_of_y_axis{"filter_pos_of_y_axis:", target_p3p, range_p3p, true,true};
	//Note: top of y axis but follow wall to neg dir
	BestTargetFilter filter_top_of_y_axis_pos{"filter_top_of_y_axis_pos", target_top_of_y_axis, range_top_of_y_axis , true};
	//Note: top of y axis but follow wall to pos dir
	BestTargetFilter filter_top_of_y_axis_neg{"filter_top_of_y_axis_neg", target_top_of_y_axis, range_top_of_y_axis , false};

	BestTargetFilter filter_short_path{"filter_short_path:", target_all, target_all, true};

	BestTargetFilter* curr_filter_{};

	BoundingBox2 map_bound;
	BoundingBox2 curr_bound{};
	Dir_t priority_dir;
	bool trend_pos{true};
	Cell_t correct_curr_;
	Cell_t origen_curr_;
#endif
private:
	int pt_;
	void adjustPosition(GridMap &map, Points &plan_path);
};
//
class WFCleanPathAlgorithm: public APathAlgorithm
{
public:
	bool generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &targets) override;
};

class SpotCleanPathAlgorithm: public APathAlgorithm
{
public:
	SpotCleanPathAlgorithm();

	bool generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &targets) override { };
	bool generatePath(GridMap &map, const Point_t &curr, bool, Points &targets, Points::iterator& ,bool& is_close);

private:

//	int spot_running_{};
	bool spot_running_{};
//	bool event_detect_{};
//	Points plan_path_remain_{};
};

class GoHomePathAlgorithm: public APathAlgorithm
{
public:
	/*
	 * @author Lin Shao Yue / Austin Liu
	 * @last modify by Austin Liu
	 *
	 * The constructor requires current clean map and stored home points and start point.
	 *
	 * @param: GridMap map, it will be copied to go_home_map_.
	 * @param: Points home_points, stored home points, size should be limited in 3.
	 * @param: Point_t start_point, the start point of this cleaning.
	 */
	GoHomePathAlgorithm() = default;
	~GoHomePathAlgorithm() = default;

	/*
	 * @author Austin Liu
	 * @last modify by Austin Liu
	 *
	 * This function is for selecting the home point and finding path to this home point.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Point_t curr, the current cell of robot.
	 * @param: int last_dir, the direction of last movement.
	 *
	 * @return: Points plan_path, the path to selected home point.
	 * @return: bool, true if generating succeeds.
	 */
	bool generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path) override;

	/*
	 * @author Austin Liu
	 *
	 * This function is for checking if robot has reached home point or start point.
	 * If it reaches home point, also remove this home point from the home point list.
	 *
	 * @param: bool should_go_to_charger, if robot reaches the target, it decides whether next state is
	 *         state go to charger.
	 *
	 * @return: bool, true if robot has reach home point or start point.
	 */
	bool reachTarget(bool &should_go_to_charger, Point_t curr);

	/*
	 * @author Austin Liu
	 *
	 * This function is for getting rest home points for clean mode.
	 *
	 * @return: Points, rest home points.
	 */
	Points getRestHomePoints();

	void resetPoints();
	/*
	 * @author Austin Liu
	 *
	 * This function is for getting current home point.
	 *
	 * @return: Point_t, current home point.
	 */
	Point_t getCurrentHomePoint();

	/*
	 * @author Austin Liu
	 *
	 * This function is for adding current point to home point list.
	 *
	 * @param: Point_t, home point to be added.
	 */
	void setHomePoint(Point_t current_point);

	Point_t getStartPoint()
	{
		return start_point_;
	}

	void updateStartPointRadian(double radian);

	bool isHomePointEmpty()
	{
		printf("home_points(%d)", home_points_.size());
		return home_points_.empty();
	}

	/*
	 * @author Austin Liu
	 *
	 * This function is for clearing blocks around home points, it can gives a better chance for robot to go to
	 * home point.
	 */
	void initForGoHomePoint(GridMap &map);

protected:

	/*
	 * @author Austin Liu
	 *
	 * This function is for erasing the target home point from the home point list.
	 *
	 * @param: Point_t target_home_point, home point to be erased.
	 *
	 * @return: bool, true if operation succeeds.
	 */
	bool eraseCurrentHomePoint();

	/*
	 * @author Austin Liu
	 *
	 * This function is for generating path to home point through the cleaned area(CLEANED in CLEAN_MAP).
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 * @param: MapDirection last_dir, the direction of last movement.
	 *
	 * @return: Cells path, the path to unclean area.
	 * @return: bool, true if operation succeeds.
	 */
	bool generatePathThroughCleanedArea(GridMap &map, const Point_t &curr, Dir_t last_dir, Cells &plan_path);

	/*
	 * @author Austin Liu
	 *
	 * This function is for generating path to home point through the cleaned area(CLEANED in CLEAN_MAP).
	 * This map has been covered by the slam map to clear the uncertain c_blocks.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 * @param: MapDirection last_dir, the direction of last movement.
	 *
	 * @return: Cells path, the path to unclean area.
	 * @return: bool, true if operation succeeds.
	 */
	bool generatePathWithSlamMapClearBlocks(GridMap &map, const Point_t &curr, Dir_t last_dir, Cells &plan_path);

	/*
	 * @author Austin Liu
	 *
	 * This function is for generating path to home point through the slam map reachable area(CLEANED in CLEAN_MAP).
	 * This map has been covered by the slam map to clear the uncertain c_blocks and add the cleanable area.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 * @param: MapDirection last_dir, the direction of last movement.
	 *
	 * @return: Cells path, the path to unclean area.
	 * @return: bool, true if operation succeeds.
	 */
	bool generatePathThroughSlamMapReachableArea(GridMap &map, const Point_t &curr, Dir_t last_dir, Cells &plan_path);

	/*
	 * @author Austin Liu
	 *
	 * This function is for generating path to home point through the uncertain area(CLEANED and UNCLEANED in CLEAN_MAP).
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 * @param: MapDirection last_dir, the direction of last movement.
	 *
	 * @return: Cells path, the path to unclean area.
	 * @return: bool, true if operation succeeds.
	 */
	bool generatePathThroughUnknownArea(GridMap &map, const Point_t &curr, Dir_t last_dir, Cells &plan_path);

	Points handleResult(bool generate_finish, Cells plan_path_cells, Point_t curr, GridMap &map);

	bool switchHomePoint(GridMap &map, const Point_t &curr);

	virtual void getNextWay(GridMap &map, const Point_t &curr);

	typedef enum
	{
		THROUGH_CLEANED_AREA = 0,
		SLAM_MAP_CLEAR_BLOCKS,
		THROUGH_SLAM_MAP_REACHABLE_AREA,
		THROUGH_UNKNOWN_AREA,
		GO_HOME_WAY_NUM
	}GoHomeWay_t;

	GoHomeWay_t home_way_index_{GoHomeWay_t::GO_HOME_WAY_NUM};
//	int home_point_index_[GO_HOME_WAY_NUM]{};
	Points home_points_{};
	Point_t start_point_{0, 0, 0};
	// current_home_point_ is initialized as an unreachable point because state go home point will check if reach home point first.
	const Point_t invalid_point_{CELL_SIZE * (MAP_SIZE + 1), CELL_SIZE * (MAP_SIZE + 1), 0};
	Point_t current_home_point_{invalid_point_};
	bool back_to_start_point_{false};
};

class FollowWallModeGoHomePathAlgorithm: public GoHomePathAlgorithm
{
public:
	FollowWallModeGoHomePathAlgorithm() = default;

private:
	void getNextWay(GridMap &map, const Point_t &curr);
};
#endif //PP_PATH_ALGORITHM_H
