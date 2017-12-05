//
// Created by austin on 17-12-3.
//

#ifndef PP_PATH_ALGORITHM_H
#define PP_PATH_ALGORITHM_H

#include <mathematics.h>
#include <deque>
#include <map.h>

typedef std::deque<Cell_t> PathType;
typedef std::deque<Cell_t> TargetList;
typedef std::deque<PathType> PathList;


class PathAlgorithm{
public:
	virtual PathType generatePath(CostMap &map, const Cell_t &curr_cell, const MapDirection &last_dir) = 0;

	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for checking the shortest and most reasonable path from start cell to target cell.
	 * The target of this function MUST be reachable. But we will use reverse algorithm to find the path,
	 * because it will have a better trace back process, the path will be more reasonable than positive
	 * algorithm.
	 *
	 * @param: CostMap map, it will use it's MAP data.(SPMAP data will be rewritten.)
	 * @param: Cell_t start, the start cell.
	 * @param: Cell_t target, the target cell.
	 * @param: MapDirection last_dir, the direction of robot last moving.
	 *
	 * @return: PathType path, the shortest path from start cell to target cell.
	 */
	PathType findShortestPath(CostMap &map, const Cell_t &start,
							  const Cell_t &target, const MapDirection &last_dir);
	/*
	 * @last modify by Austin Liu
	 *
	 * Print the path.
	 *
	 * @param: PathType path, the path from start cell to target cell.
	 */
	void displayPath(const PathType &path);

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
	 * @param: CostMap map, it will use it's MAP data.
	 * @param: PathType path, the path from start cell to target cell.
	 *
	 * @return: PathType path, an equivalent path of input path which is most far away from the obstacles.
	 */
	void optimizePath(CostMap &map, PathType &path);

	/*
	 * @author Lin Shao Yue
	 * @last modify by Austin Liu
	 *
	 * This function is for filling the path' cells with the direction towards next cell, the last one's
	 * direction is the same as its previous one.
	 *
	 * @param: PathType path, the path from start cell to target cell.
	 *
	 * @return: PathType path, the path from start cell to target cell with direction.
	 */
	void fillPathWithDirection(PathType &path);

	/*
	 * @author Lin Shao Yue
	 * @last modify by Austin Liu
	 *
	 * This function is for finding targets using Dijkstra algorithm.
	 *
	 * @param: CostMap map, it will use it's MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 *
	 * @return: true, there is reachable targets.
	 *          false, there is no reachable targets.
	 *          Cell_t target, the founded reachable target.
	 *          int cleaned_count, the cleaned grid count in map.
	 */
	bool findTargetUsingDijkstra(CostMap &map, const Cell_t& curr_cell, Cell_t& target, int& cleaned_count);

	/*
	 * @author Lin Shao Yue
	 * @last modify by Austin Liu
	 *
	 * This function is for checking if robot is trapped using Dijkstra algorithm and comparing the clean
	 * area proportion.
	 *
	 * @param: CostMap map, it will use it's MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 *
	 * @return: true, robot is trapped.
	 *          false, robot is not trapped.
	 */
	bool checkTrapped(CostMap &map, const Cell_t &curr_cell);

	/*
	 * Sorting function, for sorting paths with their targets by Y+ ascending sequence.
	 */
	static bool sortPathsWithTargetYAscend(const PathType a, const PathType b);

protected:
	Cell_t cell_direction_index[9]={{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1},{0,0}};
};

class NavCleanPathAlgorithm: public PathAlgorithm{
public:

	PathType generatePath(CostMap &map, const Cell_t &curr_cell, const MapDirection &last_dir);

private:
	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for finding path to unclean area in the same lane.
	 *
	 * @param: CostMap map, it will use it's MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 *
	 * @return: PathType path, the path to unclean area in the same lane.
	 */
	PathType findTargetInSameLane(CostMap &map, const Cell_t &curr_cell);

	/*
	 * @author Lin Shao Yue
	 * @last modify by Austin Liu
	 *
	 * This function is for finding all possiable targets in the map, judging by the boundary between
	 * cleaned and reachable unclean area.
	 *
	 * @param: CostMap map, it will use it's MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 * @param: BoundingBox2 b_map, generate by map, for simplifying code.
	 *
	 * @return: TargetList, a deque of possible targets.
	 */
	TargetList filterAllPossibleTargets(CostMap &map, const Cell_t &curr_cell, BoundingBox2 &b_map);

	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for filtering targets with their cost in the SPMAP.
	 *
	 * @param: CostMap map, it will use it's MAP data.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 * @param: TargetList possible_targets, input target list.
	 *
	 * @return: TargetList, a deque of reachable targets.
	 */
	TargetList getReachableTargets(CostMap &map, const Cell_t &curr_cell, TargetList &possible_targets);

	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for tracing the path from start cell to targets.
	 *
	 * @param: CostMap map, it will use it's MAP data.
	 * @param: TargetList target_list, input target list.
	 * @param: Cell_t start, the start cell.
	 *
	 * @return: PathList, a deque of paths from start cell to the input targets.
	 */
	PathList tracePathsToTargets(CostMap &map, const TargetList &target_list, const Cell_t& start);

	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
	 *
	 * This function is for selecting the best target in the input paths according to their path track.
	 *
	 * @param: CostMap map, it will use it's MAP data.
	 * @param: PathLIst paths, the input paths.
	 * @param: Cell_t curr_cell, the current cell of robot.
	 *
	 * @return: true if best target is selected.
	 *          false if there is no target that match the condictions.
	 *          Cell_t best_target, the selected best target.
	 */
	bool filterPathsToSelectTarget(CostMap &map, const PathList &paths, const Cell_t &curr_cell, Cell_t &best_target);
};
#endif //PP_PATH_ALGORITHM_H
