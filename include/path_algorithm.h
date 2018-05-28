//
// Created by austin on 17-12-3.
//

#ifndef PP_PATH_ALGORITHM_H
#define PP_PATH_ALGORITHM_H

#include <mathematics.h>
#include <deque>
#include "BoundingBox.h"

extern const Cell_t cell_direction_[9];
extern const Cell_t cell_direction_4[4];

typedef std::deque<Cells> PathList;

class GridMap;

class EqualTarget
{
public:
	EqualTarget(const Cell_t& curr):curr_(curr) { };
    bool operator()(const Cell_t &c_it) {
		return c_it == curr_;
//			   && std::any_of(std::begin(cell_direction_4),std::end(cell_direction_4),[&](const Cell_t& cell){ return map_.getCell(CLEAN_MAP, cell.x, cell.y) == CLEANED;});
	}

private:
	Cell_t curr_;
};

class isAccessable
{
public:
	isAccessable(const BoundingBox2& bound,GridMap* p_map):bound_(bound),p_map_(p_map) { };
	bool operator()(const Cell_t &c_it) ;

private:
	BoundingBox2 bound_;
	GridMap* p_map_;
};

class APathAlgorithm
{
public:
	virtual bool generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path) = 0;

	virtual bool checkTrapped(GridMap &map, const Cell_t &curr_cell) {return true;};

	void findPath(GridMap &map, const Cell_t &start, const Cell_t &target, Cells &path, Dir_t last_i);

	void flood_fill(const Cell_t& curr);
	using func_compare_t =  std::function<bool(const Cell_t &next)>;
	bool dijstra(GridMap& map, const Cell_t &curr_cell, Cells &targets,func_compare_t is_target,bool is_stop,func_compare_t isAccessible);
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

class IsIncrease {
public:
	IsIncrease(bool is_toward_pos):is_toward_pos_(is_toward_pos){};

	int operator()(const Cell_t &a, const Cell_t &b) {
		return is_toward_pos_ ? a.y < b.y : a.y > b.y ;
	};
private:
	bool is_toward_pos_ = true;
};

class BestTargetFilter {
public:
	BestTargetFilter(std::string name,RangeFunction target_bound_func,RangeFunction range_bound_func,  int turn_count, bool is_toward_pos,bool turn_top_limit=false):name_(name), update_target_bound(target_bound_func),update_range_bound(range_bound_func),  turn_count_(turn_count),is_toward_pos_(is_toward_pos),turn_top_limit_(turn_top_limit) {};

	void displayName(){
		std::cout << name_ << std::endl;
	};
	void updateTargetAndRangeBound(){
		target_bound = update_target_bound();
		range_bound = update_range_bound();
	};

	bool towardPos(){
		return (turn_count_== 0 && is_toward_pos_)||(turn_count_== 1 && !is_toward_pos_);
	};
//private:
//	bool is_toward_pos_=false;
	BoundingBox2 target_bound;
	BoundingBox2 range_bound;
	std::string name_;
	int turn_count_;
	bool is_toward_pos_{};
	RangeFunction update_target_bound;
	RangeFunction update_range_bound;
	bool turn_top_limit_{false};
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
	BoundingBox2 getLine(const Cell_t& curr,GridMap& map);
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
#if !USE_NEW_PATH_PLAN
    bool should_follow_wall(){
		return (		curr_filter_ == &filter_p0_1t_p
						||	curr_filter_ == &filter_p0_1t_n
						||	curr_filter_ == &filter_n0_1t_p
						||	curr_filter_ == &filter_n0_1t_n
						||	curr_filter_ == &filter_p2_0t
						||	curr_filter_ == &filter_p1_0t
						||	curr_filter_ == &filter_n2_0t
						||	curr_filter_ == &filter_n1_0t);
	};
    bool is_pox_y(){
		return curr_filter_->towardPos();
	};
#else
	bool should_follow_wall(){
		return pt_ == Y_AXIS_POS_NEXT || pt_ == Y_AXIS_NEG_NEXT || pt_ == CURR_LANE || pt_ == CURR_LANE_NEG;
	};
    bool is_pox_y(){
		return pt_ == Y_AXIS_POS_NEXT;
	};
#endif
private:
	using pair_bb = std::tuple<BoundingBox2, BoundingBox2,Dir_t>;
	std::unique_ptr<std::deque<BestTargetFilter*>> generateBounds(GridMap& map, const Cell_t& curr, int bound_i,Dir_t last_dir);

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
	 * @return: Cells, a deque of possible targets.
	 */
//	Cells filterAllPossibleTargets(GridMap &map, const Cell_t &curr_cell, BoundingBox2 &b_map);

	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for tracing the path from start cell to targets.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cells target_list, input target list.
	 * @param: Cell_t start, the start cell.
	 *
	 * @return: PathList, a deque of paths from start cell to the input targets.
	 */
//	void findPath(GridMap &map, const Cell_t &curr, const Cell_t &targets, Cells &path, int last_i);

	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for adjusting the path away from obstacles while the cost and turning count is
	 * still the same.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cells path, the path from start cell to target cell.
	 *
	 * @return: Cells path, an equivalent path of input path which is most far away from the obstacles.
	 */
	void optimizePath(GridMap &map, Cells &path,Dir_t last_dir);

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
	bool filterPathsToSelectBestPath(GridMap &map, const Cells &targets, const Cell_t &cell_curr, Cells &best_path, const Dir_t &last_dir);

	bool checkTrapped(GridMap &map, const Cell_t &curr_cell) override ;
#if !USE_NEW_PATH_PLAN
	std::unique_ptr<Cells> findTargetInSameLane(GridMap &map, const Cell_t &curr_cell);
/////////////////////////////////////////////////////////////////
	RangeFunction range_0 = [&](){
		return BoundingBox2{curr_filter_->target_bound.min - Cell_t{0,1}, curr_filter_->target_bound.max + Cell_t{0,1}};
	};

	RangeFunction range_1_2p = [&](){
		return BoundingBox2{Cell_t{curr_bound.min.x, curr_.y}, curr_filter_->target_bound.max};
	};
	RangeFunction range_1_2n = [&](){
		return BoundingBox2{curr_filter_->target_bound.min, Cell_t{curr_bound.max.x, curr_.y}};
	};
	RangeFunction range_p0_1t_p = [&](){
		return BoundingBox2{curr_bound.min - Cell_t{0,2}, Cell_t{(int16_t)(curr_.x+5), curr_.y}};
	};
	RangeFunction range_p0_1t_n = [&](){
		return BoundingBox2{curr_ - Cell_t{0,2}, Cell_t{(int16_t)(curr_.x+5), curr_.y}};
	};
	RangeFunction range_n0_1t_x = [&](){
		return BoundingBox2{Cell_t{curr_bound.min.x-5, (int16_t)(curr_.y-2)}, curr_};
	};
	RangeFunction range_n0_1t_n = [&](){
		return BoundingBox2{curr_ - Cell_t{0,2}, Cell_t{(int16_t)(curr_.x+5), curr_.y}};
	};
////////////////////////////////////////////////////////////

	RangeFunction target_p0_0t_p = [&](){
		return BoundingBox2{curr_, Cell_t{map_bound.max.x, curr_.y}};
	};

	RangeFunction target_p0_0t_n = [&](){
		return BoundingBox2{Cell_t{map_bound.min.x, curr_.y}, curr_};
	};

	RangeFunction target_p0_1t_p = [&](){
		return BoundingBox2{Cell_t{(int16_t)(curr_bound.max.x+3), curr_.y}, Cell_t{(int16_t)(curr_bound.max.x+5), curr_.y}};
	};

	RangeFunction target_p0_1t_n = [&](){
		return BoundingBox2{Cell_t{(int16_t)(curr_bound.min.x-5), curr_.y}, Cell_t{(int16_t)(curr_bound.min.x-3), curr_.y}};
	};

	RangeFunction target_p1 = [&](){
		return BoundingBox2{Cell_t{curr_bound.min.x, (int16_t)(curr_.y + 1)}, Cell_t{curr_bound.max.x, (int16_t)(curr_.y+1)}};
	};

	RangeFunction target_p2 = [&](){
		return BoundingBox2{Cell_t{curr_bound.min.x, (int16_t)(curr_.y + 2)}, Cell_t{curr_bound.max.x, (int16_t)(curr_.y + 2)}};
	};

	RangeFunction target_p3p = [&](){
		return BoundingBox2{Cell_t{map_bound.min.x, (int16_t)(curr_.y + 3)}, map_bound.max};
	};

	RangeFunction target_n1 =[&](){
		return BoundingBox2{Cell_t{map_bound.min.x, (int16_t)(curr_.y-1)}, Cell_t{map_bound.max.x, (int16_t)(curr_.y-1)}};
	};

	RangeFunction target_n2 = [&](){
		return BoundingBox2{Cell_t{curr_bound.min.x, (int16_t)(curr_.y-2)}, Cell_t{curr_bound.max.x, (int16_t)(curr_.y-2)}};
	};

	RangeFunction target_n3n =[&](){
		return BoundingBox2{map_bound.min, Cell_t{map_bound.max.x, (int16_t)(curr_.y-3)}};
	};

	RangeFunction target_all =[&](){
		return BoundingBox2{map_bound.min, map_bound.max};
	};

public:
	BestTargetFilter filter_p0_0_p{"filter_p0_0_p", target_p0_0t_p,target_p0_0t_p, 0, true};
	BestTargetFilter filter_p0_0_n{"filter_p0_0_n", target_p0_0t_n,target_p0_0t_n, 0, false};

	BestTargetFilter filter_p0_1t_p{"filter_p0_1t_p", target_p0_1t_p, range_p0_1t_p , 1, true, true};
	BestTargetFilter filter_p0_1t_n{"filter_p0_1t_n", target_p0_1t_n, range_p0_1t_n, 1, true, true};
	BestTargetFilter filter_n0_1t_p{"filter_n0_1t_p", target_p0_0t_p, range_pn_1t_p, 1, false, true};
	BestTargetFilter filter_n0_1t_n{"filter_n0_1t_n", target_p0_0t_n, range_pn_1t_n, 1, false, true};

	BestTargetFilter filter_p1_0t{"filter_p1_0t", target_p1, range_1_2p, 0, true};
	BestTargetFilter filter_p2_0t{"filter_p2_0t", target_p2, range_1_2p, 0, true};
	BestTargetFilter filter_p3p_0t{"filter_p3p_0t", target_p3p, target_p3p, 0, true};

	BestTargetFilter filter_n1_0t{"filter_n1_0t", target_n1 , range_1_2n , 0, false};
	BestTargetFilter filter_n2_0t{"filter_n2_0t", target_n2 , range_1_2n , 0, false};
	BestTargetFilter filter_n3n_0t{"filter_n3n_0t", target_n3n, target_n3n, 0, false};

//	BestTargetFilter filter_p_1t{target_all,target_all,1,true};
//	BestTargetFilter filter_n_1t{target_all, target_all, 1, false};
//	BestTargetFilter filter_p_1000t{target_all, target_all, 1000,true};
	BestTargetFilter filter_short_path{"filter_short_path:", target_all, target_all, 1000,false};
	BestTargetFilter* curr_filter_{};
//	std::deque<BestTargetFilter*> filters{};
//	std::map<int,BestTargetFilter*> planers{
//			{1,&filter_p0_0_p},
//			{2,&filter_p0_0_n},
//			{3,&filter_p0_1t_p},
//			{4,&filter_p0_1t_n},
//			{5,&filter_n0_1t_p},
//			{6,&filter_n0_1t_n},
//			{7,&filter_p3p_0t},
//			{8,&filter_p3p_0t},
//	};

	BoundingBox2 map_bound;
	BoundingBox2 bound_range;
	BoundingBox2 curr_bound{};
	Dir_t priority_dir;
	Cell_t curr_;
#endif
private:
	int pt_;
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
	bool generatePath(GridMap &map, const Point_t &curr, bool, Points &targets, const Points::iterator& );

private:

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
		return home_points_.empty();
	}

	/*
	 * @author Austin Liu
	 *
	 * This function is for clearing blocks around home points, it can gives a better chance for robot to go to
	 * home point.
	 */
	void initForGoHomePoint(GridMap &map);
private:

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

	bool switchHomePoint();

	typedef enum
	{
		THROUGH_CLEANED_AREA = 0,
		SLAM_MAP_CLEAR_BLOCKS,
		THROUGH_SLAM_MAP_REACHABLE_AREA,
		THROUGH_UNKNOWN_AREA,
		GO_HOME_WAY_NUM
	}GoHomeWay_t;

	GoHomeWay_t home_way_index_{GoHomeWay_t::THROUGH_CLEANED_AREA};
//	int home_point_index_[GO_HOME_WAY_NUM]{};
	Points home_points_{};
	Point_t start_point_{0, 0, 0};
	// current_home_point_ is initialized as an unreachable point because state go home point will check if reach home point first.
	const Point_t invalid_point_{CELL_SIZE * (MAP_SIZE + 1), CELL_SIZE * (MAP_SIZE + 1), 0};
	Point_t current_home_point_{invalid_point_};
	bool back_to_start_point_{false};
};

#endif //PP_PATH_ALGORITHM_H
