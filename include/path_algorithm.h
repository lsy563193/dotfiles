//
// Created by austin on 17-12-3.
//

#ifndef PP_PATH_ALGORITHM_H
#define PP_PATH_ALGORITHM_H

#include <mathematics.h>
#include <deque>
#include <map.h>

typedef std::deque<Cell_t> Path_t;
typedef std::deque<Cell_t> TargetList;
typedef std::deque<Path_t> PathList;


class PathAlgorithm{

public:
	virtual Path_t generatePath(GridMap &map, const Cell_t &curr_cell, const MapDirection &last_dir) = 0;

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
	 * @return: Path_t path, the shortest path from start cell to target cell.
	 */
	Path_t findShortestPath(GridMap &map, const Cell_t &start,
							  const Cell_t &target, const MapDirection &last_dir);
	/*
	 * @last modify by Austin Liu
	 *
	 * Print the path.
	 *
	 * @param: Path_t path, the path from start cell to target cell.
	 */
	void displayPath(const Path_t &path);

	/*
	 * @last modify by Austin Liu
	 *
	 * Print the targets.
	 *
	 * @param: TargetList target list.
	 */
	void displayTargets(const TargetList &target_list){};

	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for adjusting the path away from obstacles while the cost and turning count is
	 * still the same.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Path_t path, the path from start cell to target cell.
	 *
	 * @return: Path_t path, an equivalent path of input path which is most far away from the obstacles.
	 */
	void optimizePath(GridMap &map, Path_t &path);

	/*
	 * @author Lin Shao Yue
	 * @last modify by Austin Liu
	 *
	 * This function is for filling the path' cells with the direction towards next cell, the last one's
	 * direction is the same as its previous one.
	 *
	 * @param: Path_t path, the path from start cell to target cell.
	 *
	 * @return: Path_t path, the path from start cell to target cell with direction.
	 */
	void fillPathWithDirection(Path_t &path);

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
	bool checkTrapped(GridMap &map, const Cell_t &curr_cell);

	/*
	 * Sorting function, for sorting paths with their targets by Y+ ascending sequence.
	 */
	static bool sortPathsWithTargetYAscend(const Path_t a, const Path_t b);

protected:

	Cell_t cell_direction_index[9]={{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1},{0,0}};
};

typedef enum {
	THROUGH_UNKNOWN_AREA,
	THROUGH_SLAM_MAP_REACHABLE_AREA,
	THROUGH_CLEANED_AREA,
	GO_HOME_WAY_NUM
}GoHomeWay_t;

class NavGoHomePathAlgorithm: public PathAlgorithm
{
public:
	/*
	 * @author Lin Shao Yue / Austin Liu
	 * @last modify by Austin Liu
	 *
	 * The constructor requires current clean map and stored home cells.
	 *
	 * @param: GridMap map, it will be copied to go_home_map_.
	 * @param: TargetList home_cells, stored home_cells, size should be limited in 4, including
	 * home cell(0, 0).
	 *
	 * @return: Path_t path, the path to selected home cell.
	 */
	NavGoHomePathAlgorithm(GridMap &map, TargetList home_cells);
	~NavGoHomePathAlgorithm() = default;

	/*
	 * @author Lin Shao Yue / Austin Liu
	 * @last modify by Austin Liu
	 *
	 * This function is for selecting the home cell and finding path to this home cell.
	 *
	 * @param: GridMap map, it will use it's CLEAN_MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 *
	 * @return: Path_t path, the path to selected home cell.
	 */
	Path_t generatePath(GridMap &map, const Cell_t &curr_cell, const MapDirection &last_dir);

	TargetList getRestHomeCells();
private:

	GridMap go_home_map_;
	TargetList home_cells_;
	Cell_t current_home_target_;
	std::vector<int> go_home_way_list_;
	std::vector<int>::iterator go_home_way_list_it_;
};
#endif //PP_PATH_ALGORITHM_H
