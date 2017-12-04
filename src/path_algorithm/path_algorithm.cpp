//
// Created by austin on 17-12-3.
//

#include "ros/ros.h"
#include "path_algorithm/path_algorithm.h"

PathType PathAlgorithm::findShortestPath(CostMap &map, Cell_t &start, Cell_t &target, MapDirection &last_dir)
{
	PathType path_;
	path_.clear();

	int16_t x_min, x_max, y_min, y_max;
	// Get the map range.
	map.getMapRange(MAP, &x_min, &x_max, &y_min, &y_max);

	// Reset the SPMAP.
	map.reset(SPMAP);

	// Mark obstacles in SPMAP
	for (int16_t i = x_min - 1; i <= x_max + 1; ++i) {
		for (int16_t j = y_min - 1; j <= y_max + 1; ++j) {
			CellState cs = map.getCell(MAP, i, j);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				//for (m = ROBOT_RIGHT_OFFSET + 1; m <= ROBOT_LEFT_OFFSET - 1; m++)
				for (int16_t m = ROBOT_RIGHT_OFFSET; m <= ROBOT_LEFT_OFFSET; m++) {
					for (int16_t n = ROBOT_RIGHT_OFFSET; n <= ROBOT_LEFT_OFFSET; n++) {
						map.setCell(SPMAP, (i + m), (j + n), COST_HIGH);
					}
				}
			}
			else if(cs == UNCLEAN)
				map.setCell(SPMAP, i, j, COST_HIGH);
		}
	}

	// Set for start cell.
	map.setCell(SPMAP, start.X, start.Y, COST_1);

	/*
	 * Find the path to target from the start cell. Set the cell values
	 * in SPMAP to 1, 2, 3, 4 or 5. This is a method like A-Star, starting
	 * from a start point, update the cells one level away, until we reach the target.
	 */
	int16_t passValue, nextPassValue, passSet, offset;
	offset = 0;
	passSet = 1;
	passValue = 1;
	nextPassValue = 2;
	while (map.getCell(SPMAP, target.X, target.Y) == COST_NO && passSet == 1) {
		offset++;
		passSet = 0;

		/*
		 * The following 2 for loops is for optimise the computational time.
		 * Since there is not need to go through the whole costmap for seaching the
		 * cell that have the next pass value.
		 *
		 * It can use the offset to limit the range of searching, since in each loop
		 * the cell (X -/+ offset, Y -/+ offset) would be set only. The cells far away
		 * to the robot position won't be set.
		 */
		for (auto i = start.X - offset; i <= start.X + offset; i++) {
			if (i < x_min || i > x_max)
				continue;

			for (auto j = start.Y - offset; j <= start.Y + offset; j++) {
				if (j < y_min || j > y_max)
					continue;

				/* Found a cell that has a pass value equal to the current pass value. */
				if(map.getCell(SPMAP, i, j) == passValue) {
					/* Set the lower cell of the cell which has the pass value equal to current pass value. */
					if (map.getCell(SPMAP, i - 1, j) == COST_NO) {
						map.setCell(SPMAP, (int32_t) (i - 1), (int32_t) j, (CellState) nextPassValue);
						passSet = 1;
					}

					/* Set the upper cell of the cell which has the pass value equal to current pass value. */
					if (map.getCell(SPMAP, i + 1, j) == COST_NO) {
						map.setCell(SPMAP, (int32_t) (i + 1), (int32_t) j, (CellState) nextPassValue);
						passSet = 1;
					}

					/* Set the cell on the right hand side of the cell which has the pass value equal to current pass value. */
					if (map.getCell(SPMAP, i, j - 1) == COST_NO) {
						map.setCell(SPMAP, (int32_t) i, (int32_t) (j - 1), (CellState) nextPassValue);
						passSet = 1;
					}

					/* Set the cell on the left hand side of the cell which has the pass value equal to current pass value. */
					if (map.getCell(SPMAP, i, j + 1) == COST_NO) {
						map.setCell(SPMAP, (int32_t) i, (int32_t) (j + 1), (CellState) nextPassValue);
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
	if (map.getCell(SPMAP, target.X, target.Y) == COST_NO || map.getCell(SPMAP, target.X, target.Y) == COST_HIGH) {
		ROS_WARN("%s, %d: target point (%d, %d) is not reachable(%d), return -2.", __FUNCTION__, __LINE__, target.X, target.Y,
				 map.getCell(SPMAP, target.X, target.Y));
#if	DEBUG_SM_MAP
		map.print(SPMAP, target.X, target.Y);
#endif
		// Now the path_ is empty.
		return path_;
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
	int16_t i, j, m, n, tracex, tracey, tracex_last, tracey_last, totalCost = 0;
	t.X = tracex = tracex_last = target.X;
	t.Y = tracey = tracey_last = target.Y;
	path_.push_back(t);

	uint16_t next = 0;
	uint8_t dest_dir = (last_dir == MAP_POS_Y || last_dir == MAP_NEG_Y) ? 1: 0;
	//ROS_INFO("%s %d: dest dir: %d", __FUNCTION__, __LINE__, dest_dir);
	while (tracex != start.X || tracey != start.Y) {
		CellState cost_at_cell = map.getCell(SPMAP, tracex, tracey);
		auto target_cost = static_cast<CellState>(cost_at_cell - 1);

		/* Reset target cost to 5, since cost only set from 1 to 5 in the SPMAP. */
		if (target_cost == 0)
			target_cost = COST_5;

		/* Set the cell value to 6 if the cells is on the path. */
		map.setCell(SPMAP, (int32_t) tracex, (int32_t) tracey, COST_PATH);

#define COST_SOUTH	{											\
				if (next == 0 && (map.getCell(SPMAP, tracex - 1, tracey) == target_cost)) {	\
					tracex--;								\
					next = 1;								\
					dest_dir = 1;								\
				}										\
			}

#define COST_WEST	{											\
				if (next == 0 && (map.getCell(SPMAP, tracex, tracey - 1) == target_cost)) {	\
					tracey--;								\
					next = 1;								\
					dest_dir = 0;								\
				}										\
			}

#define COST_EAST	{											\
				if (next == 0 && (map.getCell(SPMAP, tracex, tracey + 1) == target_cost)) {	\
					tracey++;								\
					next = 1;								\
					dest_dir = 0;								\
				}										\
			}

#define COST_NORTH	{											\
				if (next == 0 && map.getCell(SPMAP, tracex + 1, tracey) == target_cost) {	\
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
		if (path_.back().X != tracex && path_.back().Y != tracey) {
			t.X = tracex_last;
			t.Y = tracey_last;
			path_.push_back(t);
		}
		tracex_last = tracex;
		tracey_last = tracey;
	}

	map.setCell(SPMAP, (int32_t) start.X, (int32_t) start.Y, COST_PATH);

	t.X = start.X;
	t.Y = start.Y;
	path_.push_back(t);

	std::reverse(path_.begin(),path_.end());
	displayPath(path_);

	return path_;
}

void PathAlgorithm::displayPath(const PathType& path)
{
	std::string     msg = __FUNCTION__;

	msg += " " + std::to_string(__LINE__) + ": Path size(" + std::to_string(path.size()) + "):";
	for (auto it = path.begin(); it != path.end(); ++it) {
		msg += "(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ", " + std::to_string(it->TH) + ")->";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

PathType PathAlgorithm::optimizePath(CostMap &map, PathType& path)
{
	// Optimize only if the path have more than 3 cells.
	if (path.size() > 3) {
		ROS_INFO("%s %d: Start optimize Path");
		auto it = path.begin();
		for (uint16_t i = 0; i < path.size() - 3; i++) {
			ROS_DEBUG("%s %d: i: %d, size: %ld.", __FUNCTION__, __LINE__, i, path.size());
			auto it_ptr1 = it;

			auto it_ptr2 = it_ptr1;
			it_ptr2++;

			auto it_ptr3 = it_ptr2;
			it_ptr3++;

			// Just for protection, in case the iterator overflow. It should not happen under (i < path_points.size() - 3).
			if (it_ptr3 == path.end())
				break;

			if (it_ptr2->X == it_ptr3->X) {		// X coordinates are the same for p1, p2, find a better Y coordinate.
				int16_t x_min, x_max;
				x_min = x_max = it_ptr2->X;

				int16_t	ei, ej, si, sj;
				sj = it_ptr1->X > it_ptr2->X ? it_ptr2->X : it_ptr1->X;
				ej = it_ptr1->X > it_ptr2->X ? it_ptr1->X : it_ptr2->X;
				si = it_ptr2->Y > it_ptr3->Y ? it_ptr3->Y : it_ptr2->Y;
				ei = it_ptr2->Y > it_ptr3->Y ? it_ptr2->Y : it_ptr3->Y;

				bool blocked_min, blocked_max;
				blocked_min = blocked_max = false;
				while (!blocked_min || !blocked_max) {
					for (int16_t j = si; j <= ei && (!blocked_min || !blocked_max); j++) {
						if (!blocked_min && (x_min - 1 < sj ||
								map.getCell(SPMAP, x_min - 1, j) == COST_HIGH)) {
							blocked_min = true;
						}
						if (!blocked_max && (x_max + 1 > ej ||
								map.getCell(SPMAP, x_max + 1, j) == COST_HIGH)) {
							blocked_max = true;
						}
					}
					if (!blocked_min) {
						x_min--;
					}
					if (!blocked_max) {
						x_max++;
					}
				}

				if (it_ptr3->X != (x_min + x_max) / 2) {
					it_ptr2->X = it_ptr3->X = (x_min + x_max) / 2;
				}
				//ROS_INFO("%s %d: Loop i:%d\tx_min: %d\tx_max: %d\tget x:%d", __FUNCTION__, __LINE__, i, x_min, x_max, (x_min + x_max) / 2);
			} else {
				int16_t y_min, y_max;
				y_min = y_max = it_ptr2->Y;

				int16_t	ei, ej, si, sj;
				sj = it_ptr1->Y > it_ptr2->Y ? it_ptr2->Y : it_ptr1->Y;
				ej = it_ptr1->Y > it_ptr2->Y ? it_ptr1->Y : it_ptr2->Y;
				si = it_ptr2->X > it_ptr3->X ? it_ptr3->X : it_ptr2->X;
				ei = it_ptr2->X > it_ptr3->X ? it_ptr2->X : it_ptr3->X;

				bool blocked_min, blocked_max;
				blocked_min = blocked_max = false;
				while (!blocked_min || !blocked_max) {
					for (int16_t j = si; j <= ei && (!blocked_min || !blocked_max); j++) {
						if (!blocked_min && (y_min - 1 < sj ||
								map.getCell(SPMAP, j, y_min - 1) == COST_HIGH)) {
							blocked_min = true;
						}
						if (!blocked_max && (y_max + 1 > ej ||
								map.getCell(SPMAP, j, y_max + 1) == COST_HIGH)) {
							blocked_max = true;
						}
					}
					if (!blocked_min) {
						y_min--;
					}
					if (!blocked_max) {
						y_max++;
					}
				}

				if (it_ptr3->Y != (y_min + y_max) / 2) {
					it_ptr2->Y = it_ptr3->Y = (y_min + y_max) / 2;
				}
				//ROS_INFO("%s %d: Loop i:%d\ty_min: %d\ty_max: %d\tget y:%d", __FUNCTION__, __LINE__, i, y_min, y_max, (y_min + y_max) / 2);
			}

			it++;
		}
		displayPath(path);
	} else
		ROS_INFO("%s %d:Path too short, size: %ld.", __FUNCTION__, __LINE__, path.size());

	return path;
}
