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
	 * Print the path.
	 *
	 * @param: PathType path, the path from start cell to target cell.
	 */
	void displayTargets(const TargetList &target_list){};

	/*
	 * @author Patrick Chow
	 * @last modify by Austin Liu
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
	 * @param: PathType path, the path from start cell to target cell.
	 *
	 * @return: PathType path, each cell in this path has the direction towards next cell, the last one's
	 *          direction is the same as its previous one.
	 */
	void fillPathWithDirection(PathType &path);
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
	PathType findTargetInSameLane(CostMap &map, const Cell_t &curr_cell);
	TargetList filterAllPossibleTarget(CostMap &map, const Cell_t &curr_cell, BoundingBox2 &b_map);
	TargetList getReachableTargets(CostMap &map, const Cell_t &curr_cell, TargetList &possible_targets);
	PathList tracePathsToTargets(CostMap &map, const TargetList &target_list, const Cell_t& curr_cell);
	bool filterPathsToSelectTarget(CostMap &map, const PathList &paths, const Cell_t &curr_cell, Cell_t &best_target);
};
#endif //PP_PATH_ALGORITHM_H
