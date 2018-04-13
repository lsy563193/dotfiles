//
// Created by austin on 17-12-3.
//

#ifndef PP_PATH_ALGORITHM_H
#define PP_PATH_ALGORITHM_H

#include <mathematics.h>
#include <deque>
#include <map.h>

extern const Cell_t cell_direction_[9];

typedef std::deque<Cells> PathList;

class ACleanMode;
class APathAlgorithm
{
public:
	virtual bool generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path) = 0;

	virtual bool checkTrapped(GridMap &map, const Cell_t &curr_cell) {return true;};

	Points cells_generate_points(Cells &path);
	/*
	 * @last modify by Austin Liu
	 *
	 * Print the path.
	 *
	 * @param: Cells path, the path from start cell to target cell.
	 */
	void displayCellPath(const Cells &path);

	/*
	 * @last modify by Austin Liu
	 *
	 * Print the targets.
	 *
	 * @param: Cells target list.
	 */
	void displayTargetList(const Cells &target_list);

	void displayPointPath(const Points &point_path);

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
	Cells findShortestPath(GridMap &map, const Cell_t &start,
							  const Cell_t &target, const Dir_t &last_dir, bool use_unknown,bool bound,Cell_t min_corner ,Cell_t max_corner);

	bool generateShortestPath(GridMap &map, const Point_t &curr,const Point_t &target, const Dir_t &last_dir, Points &plan_path);

	/*
	 * @
	 *
	 */
	bool isTargetReachable(GridMap map,Cell_t target);

	static ACleanMode* p_cm_;
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

typedef BoundingBox2(*RangeFunction)(Cell_t&, Cell_t&, Cell_t&);

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
	BestTargetFilter(RangeFunction range_function, int turn_count, bool is_toward_pos,bool turn_top_limit=false):range_function_(range_function), turn_count_(turn_count),is_toward_pos_(is_toward_pos),turn_top_limit_(turn_top_limit) {};

	void update(Cell_t& curr, Cell_t& min, Cell_t& max){
		auto bound = range_function_(curr, min, max);
		min_ = bound.min;
		max_ = bound.max;
	};
	int operator()(const Cells &path) {
		if(turn_count_ ==0) {
			return std::is_sorted(path.begin(), path.end(), IsIncrease(is_toward_pos_));
		}
		else if(turn_count_ == 1){
			auto point = std::is_sorted_until(path.begin(), path.end(), IsIncrease(!is_toward_pos_)) - 1;
			if(!(point != path.begin() && point != path.end()-1 && std::is_sorted(point, path.end(), IsIncrease(is_toward_pos_))))
				return false;
			if(turn_top_limit_)
			{
				if(std::abs(point->y - path.front().y) >2)
					return false;
				if(std::abs(path.back().x - path.front().x) >5)
					return false;
			}
			return true;
		}
		else/* if(turn_count_ ==-1)*/{//any turn
			return true;
		}
	};
	bool towardPos(){
		return (turn_count_== 0 && is_toward_pos_)||(turn_count_== 1 && !is_toward_pos_);
	};
//private:
//	bool is_toward_pos_=false;
	Cell_t min_;
	Cell_t max_;
	int turn_count_;
	bool is_toward_pos_{};
	RangeFunction range_function_;
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
	bool generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path) override;
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
	Cells findTargetInSameLane(GridMap &map, const Cell_t &curr_cell);

private:

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
	void findPath(GridMap &map, const Cell_t &curr, const Cell_t &targets, Cells &path, int last_i);

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
	void optimizePath(GridMap &map, Cells &path);

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

//	RangeFunction range_0_xp = [](Cell_t& curr, Cell_t& min, Cell_t& max){
//		return BoundingBox2{curr, Cell_t{max.x, curr.y}};
//	};
//	RangeFunction range_0_xn = [](Cell_t& curr, Cell_t& min, Cell_t& max){
//		return BoundingBox2{Cell_t{min.x, curr.y}, curr};
//	};
	RangeFunction range_0_xp = [](Cell_t& curr, Cell_t& min, Cell_t& max){
		return BoundingBox2{curr, Cell_t{max.x, curr.y}};
	};
	RangeFunction range_0_xn = [](Cell_t& curr, Cell_t& min, Cell_t& max){
		return BoundingBox2{Cell_t{min.x, curr.y}, curr};
	};
	RangeFunction range_p2 = [](Cell_t& curr, Cell_t& min, Cell_t& max){
		return BoundingBox2{Cell_t{min.x, (int16_t)(curr.y + 2)}, Cell_t{max.x, (int16_t)(curr.y + 2)}};
	};
	RangeFunction range_p3p = [](Cell_t& curr, Cell_t& min, Cell_t& max){
		return BoundingBox2{Cell_t{min.x, (int16_t)(curr.y + 3)}, max};
	};
	RangeFunction range_p1 = [](Cell_t& curr, Cell_t& min, Cell_t& max){
		return BoundingBox2{Cell_t{min.x, curr.y}, Cell_t{max.x, (int16_t)(curr.y+1)}};
	};
//	RangeFunction range_p_1t = [](Cell_t& curr, Cell_t& min, Cell_t& max){
//		return BoundingBox2{Cell_t{min.x, curr.y}, max};
//	};

	RangeFunction range_n2 = [](Cell_t& curr, Cell_t& min, Cell_t& max){
		return BoundingBox2{Cell_t{min.x, (int16_t)(curr.y-2)}, Cell_t{max.x, (int16_t)(curr.y-2)}};
	};
	RangeFunction range_n3n = [](Cell_t& curr, Cell_t& min, Cell_t& max){
		return BoundingBox2{min, Cell_t{max.x, (int16_t)(curr.y-3)}};
	};

	RangeFunction range_n1 = [](Cell_t& curr, Cell_t& min, Cell_t& max){
		return BoundingBox2{Cell_t{min.x, (int16_t)(curr.y-1)}, Cell_t{max.x, (int16_t)(curr.y-1)}};
	};
//	RangeFunction range_n_1t = [](Cell_t& curr, Cell_t& min, Cell_t& max){
//		return BoundingBox2{min, Cell_t{max.x, curr.y}};
//	};
	RangeFunction range_all = [](Cell_t& curr, Cell_t& min, Cell_t& max){
		return BoundingBox2{min, max};
	};
public:
//	BestTargetFilter filter_0_xp{range_0_xp, 1, true};
//	BestTargetFilter filter_0_xn{range_0_xn, 1, false};
	BestTargetFilter filter_p0_1t_xp{range_0_xp , 1, true, true};
	BestTargetFilter filter_p0_1t_xn{range_0_xn, 1, true, true};
	BestTargetFilter filter_n0_1t_xp{range_0_xp , 1, false, true};
	BestTargetFilter filter_n0_1t_xn{range_0_xn, 1, false, true};
	BestTargetFilter filter_p4p{range_p3p, 0, true};
	BestTargetFilter filter_p2{range_p2, 0, true};
	BestTargetFilter filter_p1{range_p1, 0, true};
	BestTargetFilter filter_p_1t{range_all,1,true};
	BestTargetFilter filter_n2{range_n2 , 0, false};
	BestTargetFilter filter_n4n{range_n3n, 0, false};
	BestTargetFilter filter_n1{range_n1 , 0, false};
	BestTargetFilter filter_n_1t{range_all, 1, false};
	BestTargetFilter filter_p_1000t{range_all, 1000,true};
	BestTargetFilter filter_n_1000t{range_all, 1000,false};
	BestTargetFilter* curr_filter_{};

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
	SpotCleanPathAlgorithm(float radius,Cell_t cur_cell);

	~SpotCleanPathAlgorithm();

	bool generatePath(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &targets) override;
	bool checkTrapped(GridMap &map, const Cell_t &curr_cell) override;

	void genTargets(uint8_t type,float radius, Cells *targets,const Cell_t begincell);
	void initVariables(float radius,Cell_t cur_cell);

private:

	bool spot_running_;
	Cells targets_cells_;
	Cell_t min_corner_;
	Cell_t max_corner_;
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
	GoHomePathAlgorithm(GridMap &map, Points &home_points, Point_t start_point);
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
	bool reachTarget(bool &should_go_to_charger);

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
	bool eraseHomePoint(Point_t target_home_point);

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
	bool generatePathThroughCleanedArea(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path);

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
	bool generatePathWithSlamMapClearBlocks(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path);

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
	bool generatePathThroughSlamMapReachableArea(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path);

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
	bool generatePathThroughUnknownArea(GridMap &map, const Point_t &curr, const Dir_t &last_dir, Points &plan_path);

typedef enum {
	THROUGH_CLEANED_AREA = 0,
	SLAM_MAP_CLEAR_BLOCKS,
	THROUGH_SLAM_MAP_REACHABLE_AREA,
	THROUGH_UNKNOWN_AREA,
	GO_HOME_WAY_NUM
}GoHomeWay_t;

	GoHomeWay_t home_way_index_{THROUGH_CLEANED_AREA};
	int home_point_index_[GO_HOME_WAY_NUM]{};
	Points home_points_;
//	Point_t start_point_;
	// current_home_point_ is initialized as an unreachable point because state go home point will check if reach home point first.
	Point_t current_home_point_{CELL_SIZE * (MAP_SIZE + 1), CELL_SIZE * (MAP_SIZE + 1), 0};
};
#endif //PP_PATH_ALGORITHM_H
