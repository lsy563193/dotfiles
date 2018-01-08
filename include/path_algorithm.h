//
// Created by austin on 17-12-3.
//

#ifndef PP_PATH_ALGORITHM_H
#define PP_PATH_ALGORITHM_H

#include <mathematics.h>
#include <deque>
#include <map.h>

typedef std::deque<Cells> PathList;

class APathAlgorithm
{
public:
	virtual bool generatePath(GridMap &map, const Point32_t &curr, const int &last_dir, Points &plan_path) = 0;

	virtual bool checkTrapped(GridMap &map, const Cell_t &curr_cell) = 0;

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
	 *
	 * @return: Cells path, the shortest path from start cell to target cell.
	 */
	Cells findShortestPath(GridMap &map, const Cell_t &start,
							  const Cell_t &target, const int &last_dir, bool use_unknown);

	bool generateShortestPath(GridMap &map, const Point32_t &curr,const Point32_t &target, const int &last_dir, Points &plan_path);

	/*
	 * @author Lin Shao Yue
	 * @last modify by Austin Liu
	 *
	 * This function is for finding targets using Dijkstra algorithm.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 *
	 * @return: true, there is reachable targets.
	 *          false, there is no reachable targets.
	 *          Cell_t target, the founded reachable target.
	 *          int cleaned_count, the cleaned grid count in map.
	 */
	bool findTargetUsingDijkstra(GridMap &map, const Cell_t& curr_cell, Cell_t& target, int& cleaned_count);

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

	const Cell_t cell_direction_index_[9]{{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1},{0,0}};
};

typedef enum {
	THROUGH_CLEANED_AREA,
	THROUGH_SLAM_MAP_REACHABLE_AREA,
	THROUGH_UNKNOWN_AREA,
	GO_HOME_WAY_NUM
}GoHomeWay_t;

class NavCleanPathAlgorithm: public APathAlgorithm
{
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
	 * @return: Cells path, the path to unclean area.
	 */
	bool generatePath(GridMap &map, const Point32_t &curr, const int &last_dir, Points &plan_path) override;

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
	 * @return: Cells path, the path to unclean area in the same lane.
	 */
	Cells findTargetInSameLane(GridMap &map, const Cell_t &curr_cell);

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
	Cells filterAllPossibleTargets(GridMap &map, const Cell_t &curr_cell, BoundingBox2 &b_map);

	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for filtering targets with their cost in the COST_MAP.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 * @param: Cells possible_targets, input target list.
	 *
	 * @return: Cells, a deque of reachable targets.
	 */
	Cells getReachableTargets(GridMap &map, const Cell_t &curr_cell, Cells &possible_targets);

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
	PathList tracePathsToTargets(GridMap &map, const Cells &target_list, const Cell_t& start);

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

	/*
	 * Sorting function, for sorting paths with their targets by y+ ascending sequence.
	 */
	static bool sortPathsWithTargetYAscend(const Cells a, const Cells b);

	bool checkTrapped(GridMap &map, const Cell_t &curr_cell) override ;
};
//
class WFCleanPathAlgorithm: public APathAlgorithm
{
public:
	bool generatePath(GridMap &map, const Point32_t &curr, const int &last_dir, Points &targets) override;
	bool checkTrapped(GridMap &map, const Cell_t &curr_cell) override {};
};

class SpotCleanPathAlgorithm: public APathAlgorithm
{
public:
	SpotCleanPathAlgorithm();
	SpotCleanPathAlgorithm(float diameter,Cell_t cur_cell);

	~SpotCleanPathAlgorithm();

	bool generatePath(GridMap &map, const Point32_t &curr, const int &last_dir, Points &targets) override;
	bool checkTrapped(GridMap &map, const Cell_t &curr_cell) override;

	void genTargets(uint8_t type,float diameter,Cells *targets,const Cell_t begincell);
	void initVariables(float diameter,Cell_t cur_cell);
	void giveMeCleanPoint(GridMap map,Point32_t &point);
	void refactorTargets(GridMap map,Points *targets);
private:
	
	float spot_diameter_ ;
	bool spot_running_;
	Cells targets_cells_;
	Cell_t begin_cell_; 
	Cell_t state_cell_;
};

class GoHomePathAlgorithm: public APathAlgorithm
{
public:
	/*
	 * @author Lin Shao Yue / Austin Liu
	 * @last modify by Austin Liu
	 *
	 * The constructor requires current clean map and stored home cells.
	 *
	 * @param: GridMap map, it will be copied to go_home_map_.
	 * @param: Cells home_cells, stored home_cells, size should be limited in 4, including
	 * home cell(0, 0).
	 *
	 * @return: Cells path, the path to selected home cell.
	 */
	GoHomePathAlgorithm(GridMap &map, HomePoints home_cells);
	~GoHomePathAlgorithm() = default;

	/*
	 * @author Lin Shao Yue / Austin Liu
	 * @last modify by Austin Liu
	 *
	 * This function is for selecting the home cell and finding path to this home cell.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 *
	 * @return: Cells path, the path to selected home cell.
	 */
	bool generatePath(GridMap &map, const Point32_t &curr, const int &last_dir, Points &plan_path) override;

	bool checkTrapped(GridMap &map, const Cell_t &curr_cell) override {};

	HomePoint getCurrentHomePoint();
	HomePoints getRestHomePoints();
private:

	GridMap go_home_map_;
	HomePoints home_points_;
	HomePoints rest_home_points_;
	// current_home_point_ is initialized as an unreachable point.
	HomePoint current_home_point_{{CELL_COUNT_MUL * MAP_SIZE + 1, CELL_COUNT_MUL * MAP_SIZE + 1, 0}, false};
	std::vector<int> go_home_way_list_;
	std::vector<int>::iterator go_home_way_list_it_;
};
#endif //PP_PATH_ALGORITHM_H
