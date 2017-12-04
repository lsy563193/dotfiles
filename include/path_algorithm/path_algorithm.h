//
// Created by austin on 17-12-3.
//

#ifndef PP_PATH_ALGORITHM_H
#define PP_PATH_ALGORITHM_H

#include <mathematics.h>
#include <deque>
#include <map.h>

typedef std::deque<Cell_t> PathType;

class PathAlgorithm{
public:
	virtual PathType generatePath(CostMap &map, Cell_t &curr_cell) = 0;

	// The target of this function MUST be reachable.
	PathType findShortestPath(CostMap &map, Cell_t &start, Cell_t &target, MapDirection &last_dir);

	void displayPath(const PathType& path);

	// Finding the equivalent path which is most far away from the obstacles.
	PathType optimizePath(CostMap &map, PathType& path);
};
#endif //PP_PATH_ALGORITHM_H
