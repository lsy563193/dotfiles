/**
 ******************************************************************************
 * @file	Shortest Path
 * @author	ILife Team Patrick Chau
 * @version	Ver 20160118
 * @date	18-Jan-2016
 * @brief	Function to find the shorest path to target
 ******************************************************************************
 * <h2><center>&copy; COPYRIGHT 2016 ILife CO.LTD</center></h2>
 ******************************************************************************
 */

/*
 * Functions in this file are for determining the path from the current robot postion
 * to a give target point. There are 2 version of shortest path method is include.
 *
 * In the first version, it use the A-Star like algorithm, starting from the current
 * robot position point, assign a value to the points that have an offset 1, which
 * coordinate is (x -/+ offset, y -/+ offset), then increase the offset by 1, until the
 * cell for the target point is set or there is not more value to set in the map. If the
 * cell for the target point is set, trace back the path to the robot position. This
 * method is a little bit time consuming, since the worst case would be O(3). And
 * optimization is limited.
 *
 * In the second version, it use the up-side-down tree method, the target point is
 * set to the root of the tree. Start for the target point position, construct the tree
 * by adding the line segment(a vertical line segment that without any obstcal from the map)
 * as the node that the connected to each other is the map. When the line segment includes
 * the current robot position is added into the tree, start to trace back to the root of
 * the tree. By using this method, it is trying to save the memory and computation time.
 * And by using this method, we could further optimise the searching time in the future and
 * more flexible to do with the path to the target.
 *
 * If your want to use this method for searching the shorest path, enable the define
 * SHORTEST_PATH_V2 when compile the program. The are 2 ways to store the line segments,
 * either using dynamic memory or use the static memory. It is found that, when use the
 * dynamic memory, due to the system doesn't come with the memory management machanism,
 * the program always hang when allocation memory. So it should be use with care. When
 * using the static memory, a predefined array have to be assigned, it is defined in the
 * header file ShortestPath.h, which is POS_LINE_CNT, currently, it is set to
 * (MAP_SIZE * 11 / 4), in the complicate environment, this value is not enough. It is
 * better to set with a larger value.
 */

#include <stdint.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>

#include <ros/ros.h>

#include "core_move.h"
#include "mathematics.h"
#include "shortest_path.h"

static uint8_t g_direct_go = 0;

#ifdef SHORTEST_PATH_V2

#if defined(SHORTEST_PATH_V2) && defined (SHORTEST_PATH_V2_RAM)

static LineType	*pos_line = NULL;

#else

static LineType	pos_line[POS_LINE_CNT];

#endif

static uint16_t line_cnt = 0;

extern PositionType g_pos_history[];
extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

/*
 * Free the line segments.
 *
 * @param
 *
 * @return
 */
static inline void lines_free() {

#if defined(SHORTEST_PATH_V2) && defined (SHORTEST_PATH_V2_RAM)

	if (line_cnt > 0) {
		memset(pos_line, 0, sizeof(LineType) * line_cnt);
		free(pos_line);
		line_cnt = 0;
		pos_line = NULL;
	}

#else

	line_cnt = 0;

#endif
}

/*
 * Initialization for searching shortest path by using the up-side-down tree.
 *
 * @param dg	direction to go
 *
 * @return
 */
void path_position_init(uint8_t dg)
{
	g_direct_go = dg;

#if defined(SHORTEST_PATH_V2) && defined (SHORTEST_PATH_V2_RAM)

	/*
	 * If defined to use dynamic memory allocation for line segments,
	 * release the memory and reset the point to NULL.
	 */
	if (pos_line != NULL) {
		memset(pos_line, 0, sizeof(LineType) * line_cnt);
		line_cnt = 0;
		free(pos_line);
	}
	pos_line = NULL;

#endif

	line_cnt = 0;
}

/*
 * Display all the line segments for debugging.
 *
 * @param
 *
 * @return
 */
void path_line_dump()
{
#ifdef DEBUG_POS
	int16_t i;

	printf("\tLevel\tY\tX\n");
	for (i = 0; i < line_cnt; i++) {
		printf("%d:\t%d\t%d\t%d - %d\n", i, pos_line[i].level, pos_line[i].y, pos_line[i].x, pos_line[i].x + pos_line[i].x_off);
	}
#endif

	printf("Line List: %d\n", line_cnt);
}

/*
 * Get the total number of line segments
 *
 * @param
 *
 * @return	lines count
 */
uint16_t path_line_get_count()
{
	return line_cnt;
}

/*
 * Insert a line if the total number of line segments are less then the POS_LINE_CNT
 *
 * @param x	starting X coordinate of the line segment
 * @param x_off	offset from the starting x coordinate
 * @param y	y coordinate of the line segment
 *
 * @return	a value of 0 if the line segment fails to insert.
 * 		a value of 1 if the line segment is inseted.
 */
uint8_t  path_line_insert(int16_t x, int16_t x_off, int16_t y)
{
	/* FIXME: check memory allocation. */

	if (line_cnt + 1 >= POS_LINE_CNT) {
		printf("%s %d: too many lines inserted %d\n", __FUNCTION__, __LINE__, line_cnt);
		return 0;
	}

	line_cnt++;

#if defined(SHORTEST_PATH_V2) && defined (SHORTEST_PATH_V2_RAM)

	/* If dynamic memory is used, allocate the memory for the new line segment. */
	pos_line = realloc(pos_line, sizeof(LineType) * line_cnt);

#endif

	pos_line[line_cnt - 1].x = x;
	pos_line[line_cnt - 1].x_off = x_off;
	pos_line[line_cnt - 1].y = y;
	pos_line[line_cnt - 1].level = 0;
	return 1;
}

/*
 * Check a line segment has been inserted or not.
 *
 * @param x	starting X coordinate of the line segment
 * @param x_off	offset from the starting X coordinate
 * @param y	y coordinate of the line segment
 *
 * @return	a value of 0 if line segment is not inserted
 * 		a value of 1 if line segment is inserted
 */
uint8_t path_line_is_inserted(int16_t x, int16_t x_off, int16_t y)
{
	int i;
	uint8_t	retval = 0;

	for (i = 0; retval == 0 && i < line_cnt; i++) {
		if (y == pos_line[i].y && pos_line[i].x <= x && (x + x_off ) <= (pos_line[i].x + pos_line[i].x_off)) {
			retval = 1;
		}
	}
	return retval;
}

/*
 * Processing a new line segment by give the start/end X coordinate and the Y coordinate
 *
 * @param x1	starting X coordinate of the line segment
 * @param x2	ending X coordinate of the line segment
 * @param y	y coordinate of the line segment
 *
 * @return	a value of 0 if it fails to process the new line segment
 * 		a value of 1 if the line segment is process successfully
 */
uint8_t  path_line_process(int16_t x1, int16_t x2, int16_t y)
{
	int	i;
	uint8_t	start;
	int16_t	p1x, p2x, p3x;

	//printf("%s %d: X: %d - %d\tY: %d\n", __FUNCTION__, __LINE__, x1, x2, y);
	p1x = x1;
	p2x = x2;

	/*
	 * Check for the closest point of the giving starting X coordinate towards
	 * the POS_X(X coordinate should be increasing) direction which is accessible.
	 */
	do {
		if (is_block_accessible(p1x, y) == 0) {
			p1x++;
		} else {
			break;
		}
	} while (p1x <= p2x);

	/*
	 * Check for the closest point of the giving ending X coordinate towards
	 * the NEG_X(X coordinate should be decreasing) direction which is accessible.
	 */
	do {
		if (is_block_accessible(p2x, y) == 0) {
			p2x--;
		} else {
			break;
		}
	} while (p1x <= p2x);
	//printf("%s %d: X: %d - %d\tY: %d\n", __FUNCTION__, __LINE__, p1x, p2x, y);

	/* starting point is greater than ending point, skip. */
	if (p1x > p2x) {
		return 1;
	}

	//printf("%s %d: X: %d - %d\tY: %d\n", __FUNCTION__, __LINE__, p1x, p2x, y);

	/* Handling the line segment is breoken into small segments. All those segments should be inserted. */
	start = 1;
	p3x = p1x;

	//printf("%s %d: inserting line: X: %d - %d\tY: %d\n", __FUNCTION__, __LINE__, p1x, p2x, y);
	for (i = p1x; i <= p2x; i++) {
		if (i == p2x) {
			/* process the last point. */
			p1x = (start == 0 ? p2x : p1x);
			p3x = (start == 0 ? p2x : p3x);

			/* Check the line segment before insert. */
			if (path_line_is_inserted(p1x, (is_block_accessible(i, y) == 1) ? p2x - p1x : p3x - p1x, y) == 1) {
				continue;
			}
			if (path_line_insert(p1x, (is_block_accessible(i, y) == 1) ? p2x - p1x : p3x - p1x, y) == 0) {
				return 0;
			}
			//printf("%s %d: inserting line: i: %d\tp2.X: %d\tX: %d - %d\tY: %d\n", __FUNCTION__, __LINE__, i, p2.X, p1.X,  p3.X, p1.Y);
		} else if (is_block_accessible(i, y) == 1) {
			/* If the current point is accessible, adjust the end of the current segment. */
			p3x = i;
			if (start == 0) {
				p1x = p3x;
				start = 1;
			}
		} else {
			if (start == 1) {
				start = 0;

				/* Check the line segment before insert. */
				if (path_line_is_inserted(p1x, p3x - p1x, y) == 1) {
					continue;
				}
				if (path_line_insert(p1x, p3x - p1x, y) == 0) {
					return 0;
				}
				//printf("%s %d: inserting line: i: %d\tp2x: %d\tX: %d - %d\tY: %d\n", __FUNCTION__, __LINE__, i, p2x, p1x,  p3x, y);
			}
		}
	}
	return 1;
}

/*
 * Update the levels of the current line segments so far
 *
 * @param line_idx	the index of the target line segment in the line list
 * @param cur_idx	the index of the line segment that the robot currently in
 *
 * @return
 */
void path_trace_path(int16_t line_idx, int16_t cur_idx)
{
	int16_t	i, j;
	uint8_t	level, level_next, level_set;

	/* Reset the level of the line segments in the line list. */
	for (i = 0; i < line_cnt; i++) {
		pos_line[i].level = 0;
	}

	/*
	 * Loop for updating the level of the line segments. Stops when no level is set.
	 * The target line segment always has the level of 1.
	 */
	level_next = 2;
	level = level_set = 1;
	pos_line[line_idx].level = level;
	while (level_set == 1) {
		level_set = 0;

		//printf("%s %d: cnt: %d\tlevel: %d\n", __FUNCTION__, __LINE__, line_cnt, level);
		for (i = 0; i < line_cnt; i++) {
			/* Skip if the line segment is not the current target line segment. */
			if (pos_line[i].level != level) {
				continue;
			}

			for (j = 0; j < line_cnt; j++) {
				/* Skip if the level of the line segment is set. */
				if (pos_line[j].level != 0) {
					continue;
				}

				/* Skip the line segment is not conneted to the current target line segment. */
				if ((pos_line[i].x + pos_line[i].x_off) < pos_line[j].x || pos_line[i].x > (pos_line[j].x + pos_line[j].x_off)) {
					continue;
				}

				/* Update the line segment's level if it is next to the current target line segment. */
				if (abs(pos_line[i].y - pos_line[j].y) == 1) {
					pos_line[j].level = level_next;
					level_set = 1;
				}
			}
#if 0
			printf("\tX\t\tY\tlevel\n");
			for (i = 0; i < line_cnt; i++) {
				printf("%d:\t%d - %d\t\t%d\t%d\n", i, pos_line[i].x, pos_line[i].x + pos_line[i].x_off, pos_line[i].y, pos_line[i].level);
			}
#endif
		}

		/* Stop when reached the line that the robot is currently in. */
		if (pos_line[cur_idx].level != 0) {
			break;
		}

		level++;
		level_next++;
	}
}

/*
 * By given a target, find the path to the target.
 * In this version, a up-side-down tree is built for the path searching, the target will always be
 * the root of the tree, the nodes are the line segments that can directly reach each other, a level
 * is set for each node, and the level of the target will always be 1, when the line segment of the
 * robot position is found, and the level of that line segment is set, start to trace the path back
 * to the root, which is the target.
 *
 * @param pos	The current robot position
 * @param x	The target X Coordinate that the robot wants to go
 * @param y	The target Y Coordinate that the robot wants to go
 * @param *x_next	The next X Coordinate that the robot should go before reaching the target
 * @param *y_next	The next Y Coordinate that the robot should go before reaching the target
 *
 * @return	-2: Robot is trapped
 * 		-1: Path to target is not found
 * 		1:  Path to target is found
 */
int16_t path_move_to_unclean_area(Cell_t pos, int16_t x, int16_t y, int16_t *x_next, int16_t *y_next)
{
	uint8_t	level_cur, level_target, level_next, level_set, should_trace, found, blocked;

	int16_t	line_idx, cur_idx, next_idx;
	int16_t	i, j, k, dist, dist_min, x_tmp, y_tmp, offset, x1, x2, x_pos, y_pos, x_min, x_max, y_min, y_max;

	x_pos = pos.X;
	y_pos = pos.Y;
	x_min = g_x_min;
	x_max = g_x_max;
	y_min = g_y_min;
	y_max = g_y_max;

	printf("%s %d: (%d, %d) (%d, %d)\n", __FUNCTION__, __LINE__, x, y, *x_next, *y_next);

	/*
	 * Construct all the line segments base on the current X/Y range of the Map.
	 * It starts from the target position that the robot wants to go. Once the
	 * line segment of the current robot position is reached, starts to trace the
	 * path to the target, if the path is found, stops constructing the line
	 * segments.
	 */
	line_idx = cur_idx = -1;
	offset = should_trace = 0;
	y_tmp = (y + y_pos) / 2;
	while (((y_tmp - offset) >= y_min - 2) || ((y_tmp + offset) <= y_max + 2)) {
		/*
		 * Process right hand side of the target position.
		 * Stop when the lower limit of Y Coordinate is reach.
		 * The lower limit is set to (y_min - 2), can't set to y_min,
		 * otherwise, robot fails to move sometimes.
		 */
		if ((y_tmp - offset) >= y_min - 2) {
			if (path_line_process(x_min - 1, x_max + 1, y_tmp - offset) == 0) {
				break;
			}
		}

		/*
		 * Process left hand side of the target position.
		 * Stop when the upper limit of Y Coordinate is reach.
		 * The upper limit is set to (y_max + 2), can't set to y_max,
		 * otherwise, robot fails to move sometimes.
		 *
		 * When offset is 0, y - offset = y + offset, so skip it.
		 */
		if (offset != 0 && (y_tmp + offset) <= y_max + 2) {
			if (path_line_process(x_min - 1, x_max + 1, y_tmp + offset) == 0) {
				break;
			}
		}

		/*
		 * Check for the current robot position has been processed or not,
		 * if it is processed, enable path tracing.
		 */
		if ((y_tmp - offset) == (y > y_pos ? y_pos : y) - 3 || (y_tmp + offset) == (y > y_pos ? y : y_pos) + 3) {
			should_trace = 1;

			/* If robot is currently blocked, try to find the nearest point in the current lane to move back. */
			if (is_block_accessible(x_pos, y_pos) == 0) {
				j = dist_min = SHRT_MAX;
				for (i = line_cnt - 1; i >= 0; i--) {
					if (y_pos != pos_line[i].y) {
						continue;
					}
					printf("%s %d: x: %d\toffset: %d\ty: %d\tdist_min: %d\n",
						__FUNCTION__, __LINE__, pos_line[i].x, pos_line[i].x_off, pos_line[i].y, dist_min);

					if (abs(pos_line[i].x - x_pos) < dist_min) {
						dist_min = abs(pos_line[i].x - x_pos);
						j = pos_line[i].x;
					}
					if (abs((pos_line[i].x + pos_line[i].x_off) - x_pos) < dist_min) {
						dist_min = abs(pos_line[i].x + pos_line[i].x_off - x_pos);
						j = pos_line[i].x + pos_line[i].x_off;
					}
				}

				path_line_dump();
				if (dist_min != SHRT_MAX && j != SHRT_MAX) {
					printf("%s %d: j: %d\tx_pos: %d\n", __FUNCTION__, __LINE__, j, x_pos);
					if (j == x_pos) {
						printf("%s %d: both front & back are blocked!\n", __FUNCTION__, __LINE__);
						lines_free();
						return -2;
					} else {
						blocked = 0;
						x1 = x_pos > j ? j : x_pos;
						x2 = x_pos > j ? x_pos : j;
						for (i = x1; blocked == 0 && i <= x2; i++) {
							if (is_block_accessible(i, y_pos) == 0) {
								blocked = 1;
							}
						}

						if (blocked == 0) {
							*x_next = j;
							*y_next = y_pos;
							printf("%s %d: moving back to (%d, %d)!\n", __FUNCTION__, __LINE__, *x_next, *y_next);
							lines_free();
							return SCHAR_MAX;
						} else if (blocked == 1 && (is_block_accessible(x_pos, y_pos - 1) == 1 || is_block_accessible(x_pos, y_pos + 1) == 1)) {
							*x_next = x_pos;
							*y_next = y_pos + ((is_block_accessible(x_pos, y_pos - 1) == 1) ? -1 : 1);
							printf("%s %d: moving to left/right (%d, %d)!\n", __FUNCTION__, __LINE__, *x_next, *y_next);

							lines_free();
							return 1;
						} else {
							printf("%s %d: no way to move, front/back/left/right are blocked!\n", __FUNCTION__, __LINE__);
							lines_free();
							return -2;
						}
					}
				}
				printf("%s %d: can't find nearest point to current g_pos_history!\n", __FUNCTION__, __LINE__);
			}
		}
		/*
		 * Both robot position & target position are in the line list,
		 * can try to trace the path in order to minimize the processing time.
		 */
		if (should_trace == 1) {
			/* Get the line segment that the target is currently in. */
			if (line_idx == -1 ) {
				for (i = 0; i < line_cnt && line_idx == -1; i++) {
					if (pos_line[i].x <= x && x <= (pos_line[i].x + pos_line[i].x_off) && y == pos_line[i].y) {
						line_idx = i;
					}
				}
			}
			/* Get the line segment that the robot is currently in. */
			if (cur_idx == -1 ) {
				for (i = 0; i < line_cnt && cur_idx == -1; i++) {
					if (pos_line[i].x <= x_pos && x_pos <= (pos_line[i].x + pos_line[i].x_off) && y_pos == pos_line[i].y) {
						cur_idx = i;
					}
				}

				/* If not found, robot is trapped. */
				if ( cur_idx == -1) {
					printf("%s %d: can't locate the curent line\n", __FUNCTION__, __LINE__);
					*x_next = x_pos;
					*y_next = y_pos;

					path_line_dump();
					lines_free();
					return -2;
				}
			}

			/* If both line segments of robot position & target position are found, try to trace the path. */
			if (line_idx != -1 && cur_idx != -1) {
				path_trace_path(line_idx, cur_idx);

				/* When both line segment's levels are not 0, the path should be ready. */
				if (pos_line[line_idx].level != 0 && pos_line[cur_idx].level != 0) {
					break;
				}
			}
			//printf("%s %d: line cnt: %d\t line_idx: %d\tcur_idx: %d\n", __FUNCTION__, __LINE__, line_cnt, line_idx, cur_idx);
		}

		offset++;
	}

	/*
	 * Try to find the line segment that closest to the target position.
	 * In many case, the closest line to the target should have a distance 0.
	 */
	line_idx = -1;
	dist_min = SHRT_MAX;
	for (i = line_cnt - 1; i >= 0; i--) {
		dist = distance2line(pos_line[i].x, pos_line[i].y, pos_line[i].x + pos_line[i].x_off, pos_line[i].y, x, y);

#if 0
		printf("%s %d:\tmin dist: %d\tabs dist: %d(%d)\tp1: (%d, %d)\tp2: (%d, %d)\tdest:(%d, %d)\n",
			__FUNCTION__, __LINE__, dist_min, abs(dist), dist, pos_line[i].x, pos_line[i].y, pos_line[i].x + pos_line[i].x_off, pos_line[i].y, x, y);
#endif

		/* Possible line segment that can reach target point. */
		if (abs(dist) < dist_min) {
			/* Check whether the target point is located in the possible line segment. */
			if ((pos_line[i].x <= x && x <= pos_line[i].x + pos_line[i].x_off)) {
				dist_min = abs(dist);
				//printf("%s %d:\tmin dist: %d\n", __FUNCTION__, __LINE__, dist_min);
				line_idx = i;
			}
		} else if (abs(dist) == dist_min) {
			if ((pos_line[i].x <= x && x <= pos_line[i].x + pos_line[i].x_off) && abs(x_pos - pos_line[line_idx].y) > abs(x_pos - pos_line[i].y)) {
				dist_min = abs(dist);
				//printf("%s %d:\tmin dist: %d\n", __FUNCTION__, __LINE__, dist_min);
				line_idx = i;
			}
		}
	}
	printf("%s %d:\tlines cnt: %d\tidx: %d (dist: %d)\n", __FUNCTION__, __LINE__, line_cnt, line_idx, dist_min);

	if (dist_min != 0) {
		printf("%s %d:\twarning, not direct line that target is on\n", __FUNCTION__, __LINE__);
		line_idx = -1;
	}

	/* It is impossible to find the line segment that can reach the target. */
	if (line_idx < 0) {
		printf("%s %d: can't locate the dest line\n", __FUNCTION__, __LINE__);
		*x_next = x_pos;
		*y_next = y_pos;

		path_line_dump();
		lines_free();
		return -1;
	}
	printf("%s %d: cur idx: %d\n", __FUNCTION__, __LINE__, cur_idx);

	/*
	 * Robot is already in the target row, find how far the robot can move.
	 */
	if (x_pos == x && y_pos != y) {
		x_tmp = x_pos;
		y_tmp = y_pos;
		offset = y_pos > y ? -1 : 1;
		for (i = y_pos + offset; ; i += offset) {
			if (is_block_accessible(x, i) == 0) {
				printf("%s %d: (%d, %d) is unaccessible\n", __FUNCTION__, __LINE__, x, i);
				break;
			}

			y_tmp = i;
			if (i == y) {
				break;
			}
		}

		/* Try to avoid repeatly hit the obstcal ahead. */
		if (g_pos_history[0].x == g_pos_history[1].x && g_pos_history[0].y == g_pos_history[1].y) {
			path_trace_path(line_idx, cur_idx);
			path_line_dump();

			if (pos_line[cur_idx].x_off > 1) {
				/* Possibly ahead is blocked */
				*x_next = (pos_line[cur_idx].x + pos_line[cur_idx].x + pos_line[cur_idx].x_off) / 2;
				*y_next = y_pos;
				printf("%s %d: cur idx: %d\tx1: %d\tx2: %d\tdest: (%d, %d)\n",
						__FUNCTION__, __LINE__, cur_idx, pos_line[cur_idx].x, pos_line[cur_idx].x + pos_line[cur_idx].x_off, *x_next, *y_next);
				lines_free();
				return 1;
			}
		}

		printf("%s %d: %d %d\n", __FUNCTION__, __LINE__, x_tmp, y_tmp);
		/* Found & can reach directly. */
		if (y == y_tmp) {
			*x_next = x_tmp;
			*y_next = y_tmp;
			lines_free();
			return 1;
		}
	}

	/* If can't reach directly, using the construct line segments to trace the path. */
	path_trace_path(line_idx, cur_idx);
	path_line_dump();

	level_cur = pos_line[cur_idx].level;
	level_target = pos_line[line_idx].level;

	/* If the target position and the robot position have the same level, try to go directly. */
	if (level_cur == level_target) {
		if (x_pos != x && y_pos != y) {
			*x_next = x;
			*y_next = y_pos;
		} else {
			*x_next = x;
			*y_next = y;
		}
		printf("%s %d: %d %d, level: %d(%d)\n", __FUNCTION__, __LINE__, *x_next, *y_next, level_target, level_cur);
		lines_free();
		return 1;
	}

	/*
	 * Loop for trace the path by using the up-side-down tree that constructed.
	 * Starting from the line segment that the robot is in, and trace back to the
	 * root of the tree.
	 */
	int walk_horizontally = 0;
	next_idx = cur_idx;
	level_next = level_cur - 1;
	x_tmp = x_pos;
	y_tmp = y_pos;
	printf("%s %d: next_idx: %d\tcur_idx: %d\tlevel_next: %d\tlevel_cur: %d\n", __FUNCTION__, __LINE__, next_idx, cur_idx, level_next, level_cur);
	while (level_next) {
		level_set = 0;
		for (i = 0; i < line_cnt && level_next != 0; i++) {
			/* Skip if the level of the line segment is not our target level. */
			if (pos_line[i].level != level_next) {
				continue;
			}

			/* Skip if the line segment is not next to the current target line segment. */
			if (abs(pos_line[i].y - pos_line[next_idx].y) != 1) {
				continue;
			}

			/* Skip if the line segment can't be reached by using the current target line segment. */
			if (pos_line[i].x > (pos_line[next_idx].x + pos_line[next_idx].x_off) || (pos_line[i].x + pos_line[i].x_off) < pos_line[next_idx].x) {
				continue;
			}

			if (pos_line[i].y != pos_line[next_idx].y) {
				//printf("%s %d: X: %d\tidx: %d\tX1: %d\tX2: %d\n", __FUNCTION__, __LINE__, x_pos, i, pos_line[i].x, pos_line[i].x + pos_line[i].x_off);
				if (walk_horizontally == 0 && x_pos > pos_line[i].x + pos_line[i].x_off) {
					/*
					 * If the robot X coordinate is greate than the next line segment's ending X coordinate.
					 * Set that as the entrance point to get into that line segment. If the line segment
					 * is too short, try to get set the middle of the line segment as the entrance point,
					 * this is to avoid hitting obstcal.
					 */
					//*x_next = pos_line[i].x + pos_line[i].x_off;

					*y_next = y_tmp;
#if 0
					x1 = pos_line[i].x > pos_line[next_idx].x ? pos_line[i].x : pos_line[next_idx].x;
					x2 = pos_line[i].x + pos_line[i].x_off;
					if ((pos_line[i].x + pos_line[i].x_off) > (pos_line[next_idx].x + pos_line[next_idx].x_off)) {
						x2 = pos_line[next_idx].x + pos_line[next_idx].x_off;
					}

					*x_next = (x2 - x1 <= 6) ? (x1 + x2) / 2 : pos_line[i].x + pos_line[i].x_off;
#else
					k = next_idx;
					found = 1;
					x_min = pos_line[k].x;
					x_max = pos_line[k].x + pos_line[k].x_off;
					while (found) {
						level_set = 0;
						for (j = 0; level_set == 0 && j < line_cnt; j++) {
							if (pos_line[j].level == 0) {
								continue;
							}
							if (abs(pos_line[k].y - pos_line[j].y) != 1) {
								continue;
							}
							if (pos_line[k].level - 1 != pos_line[j].level) {
								continue;
							}
							if (pos_line[k].x > (pos_line[j].x + pos_line[j].x_off) || (pos_line[j].x + pos_line[j].x_off) < pos_line[k].x) {
								continue;
							}
							if (x_tmp >= (pos_line[j].x + pos_line[j].x_off)) {
								if ((pos_line[next_idx].y - pos_line[i].y) == (pos_line[k].y - pos_line[j].y)) {
									if ((pos_line[j].x + pos_line[j].x_off) < x_min || pos_line[j].x > x_max ) {
										continue;
									}
									x_tmp = pos_line[j].x + pos_line[j].x_off;
									level_set = 1;
									k = j;
									if (x_min < pos_line[j].x) {
										x_min = pos_line[j].x;
									}
									if (x_max > (pos_line[j].x + pos_line[j].x_off)){
										x_max = pos_line[j].x + pos_line[j].x_off;
									}
								} else {
									break;
								}
							}
						}
						if (level_set == 0) {
							found = 0;
						}
					}
					printf("%s %d: k: %d next idx: %d level: %d min: %d max: %d\n", __FUNCTION__, __LINE__, k, next_idx, pos_line[k].level, x_min, x_max);
					if (k != next_idx) {
						if (1 || pos_line[k].level == 1) {
							if (x >= x_min && x <= x_max) {
								*x_next = x;
							} else if (x < x_min) {
								*x_next = x_min;
							} else if (x > x_max) {
								*x_next = x_max;
							} else {
								if (x_max - x_min < 6) {
									*x_next = (x_min + x_max) / 2;
								} else if (x_min > x) {
									*x_next = x_min + 1;
								} else {
									*x_next = x_max - 1;
								}
							}
						} else {
							//*x_next = pos_line[k].x + pos_line[k].x_off;
							*x_next = (x_min + x_max) / 2;
						}
					} else {
						x1 = pos_line[i].x > pos_line[next_idx].x ? pos_line[i].x : pos_line[next_idx].x;
						x2 = pos_line[i].x + pos_line[i].x_off;
						if ((pos_line[i].x + pos_line[i].x_off) > (pos_line[next_idx].x + pos_line[next_idx].x_off)) {
							x2 = pos_line[next_idx].x + pos_line[next_idx].x_off;
						}

						*x_next = (x2 - x1 <= 6) ? (x1 + x2) / 2 : pos_line[i].x + pos_line[i].x_off;
					}
#endif
					printf("%s %d: %d %d\n", __FUNCTION__, __LINE__, *x_next, *y_next);
					lines_free();
					return 1;
				} else if (walk_horizontally == 0 && x_pos < pos_line[i].x) {
					/*
					 * If the robot X coordinate is greate than the next line segment's ending X coordinate.
					 * Set that as the entrance point to get into that line segment. If the line segment
					 * is too short, try to get set the middle of the line segment as the entrance point,
					 * this is to avoid hitting obstcal.
					 */
					//*x_next = pos_line[i].x;

					*y_next = y_tmp;
#if 0
					x1 = pos_line[i].x > pos_line[next_idx].x ? pos_line[i].x : pos_line[next_idx].x;
					x2 = pos_line[i].x + pos_line[i].x_off;
					if ((pos_line[i].x + pos_line[i].x_off) > (pos_line[next_idx].x + pos_line[next_idx].x_off)) {
						x2 = pos_line[next_idx].x + pos_line[next_idx].x_off;
					}

					*x_next = (x2 - x1 <= 6) ? (x1 + x2) / 2 : pos_line[i].x;
#else
					k = next_idx;
					found = 1;
					x_min = pos_line[k].x;
					x_max = pos_line[k].x + pos_line[k].x_off;
					while (found) {
						level_set = 0;
						for (j = 0; level_set == 0 && j < line_cnt; j++) {
							if (pos_line[j].level == 0) {
								continue;
							}
							if (abs(pos_line[k].y - pos_line[j].y) != 1) {
								continue;
							}
							if (pos_line[k].level - 1 != pos_line[j].level) {
								continue;
							}
							if (pos_line[k].x > (pos_line[j].x + pos_line[j].x_off) || (pos_line[j].x + pos_line[j].x_off) < pos_line[k].x) {
								continue;
							}
							if (x_tmp <= pos_line[j].x) {
								if ((pos_line[next_idx].y - pos_line[i].y) == (pos_line[k].y - pos_line[j].y)) {
									if ((pos_line[j].x + pos_line[j].x_off) < x_min || pos_line[j].x > x_max ) {
										continue;
									}

									x_tmp = pos_line[j].x;
									level_set = 1;
									k = j;
									if (x_min < pos_line[j].x) {
										x_min = pos_line[j].x;
									}
									if (x_max > (pos_line[j].x + pos_line[j].x_off)){
										x_max = pos_line[j].x + pos_line[j].x_off;
									}
								} else {
									break;
								}
							}
						}
						if (level_set == 0) {
							found = 0;
						}
					}
					printf("%s %d: k: %d next idx: %d level: %d min: %d max: %d\n", __FUNCTION__, __LINE__, k, next_idx, pos_line[k].level, x_min, x_max);
					if (k != next_idx) {
						if (1 || pos_line[k].level == 1) {
							if (x >= x_min && x <= x_max) {
								*x_next = x;
							} else if (x < x_min) {
								*x_next = x_min;
							} else if (x > x_max) {
								*x_next = x_max;
							} else {
								if (x_max - x_min < 6) {
									*x_next = (x_min + x_max) / 2;
								} else if (x_min > x) {
									*x_next = x_min + 1;
								} else {
									*x_next = x_max - 1;
								}
							}
						} else {
							//*x_next = pos_line[k].x;
							*x_next = (x_min + x_max) / 2;
						}
					} else {
						x1 = pos_line[i].x > pos_line[next_idx].x ? pos_line[i].x : pos_line[next_idx].x;
						x2 = pos_line[i].x + pos_line[i].x_off;
						if ((pos_line[i].x + pos_line[i].x_off) > (pos_line[next_idx].x + pos_line[next_idx].x_off)) {
							x2 = pos_line[next_idx].x + pos_line[next_idx].x_off;
						}

						*x_next = (x2 - x1 <= 6) ? (x1 + x2) / 2 : pos_line[i].x;
					}
#endif
					printf("%s %d: %d %d\n", __FUNCTION__, __LINE__, *x_next, *y_next);
					lines_free();
					return 1;
				} else {
					walk_horizontally = 1;
					/*
					 * When moving horizontally, it should stop when the robot position is greater X2,
					 * or less than X1 of the next line target line segment, these are the same case as above.
					 */
					if (x_pos > (pos_line[i].x + pos_line[i].x_off)  || x_pos < pos_line[i].x) {

						/*
						 * Below it is to make sure, that we can't go further horizontally, since the line
						 * segment that have the same Y coordnate, might have the save level. Due to the line
						 * segments are stored in an array, the first line segment that has the same level
						 * of the target level will be process, this will cause the path is not optimal.
						 * So, it is better to loop the line list again and search for the best path.
						 *
						 * 			Y
						 * 			|
						 * 			| (line segment 1)
						 * 			|
						 *
						 * 			|
						 * 		R	| (line segment 2)
						 * 			|
						 *
						 *  As in above, the robot (R) wants to go through Y, in the line segment list, if the index
						 *  of segment 1 is I, index of segment 2 is (I + 1), the shortest way to go through Y is
						 *  segment 2, but not segment 1.
						 *
						 */
						found = 0;
						printf("%s %d: %d %d %d\n", __FUNCTION__, __LINE__, pos_line[i].y, i, next_idx);
						for (j = 0; j < line_cnt; j++) {
							/* Skip it the line segment has a different Y coordinate. */
							if (pos_line[i].y != pos_line[j].y) {
								continue;
							}

							/* Skip it the line segment has a different level. */
							if (pos_line[i].level != pos_line[j].level) {
								continue;
							}

							/* Skip it the line segment can't be reach by the current target line segment. */
							if (pos_line[j].x > (pos_line[next_idx].x + pos_line[next_idx].x_off) || (pos_line[j].x + pos_line[j].x_off) < pos_line[next_idx].x) {
								continue;
							}

							/* Found it, it can go through horizontally. */
							if (x_pos <= (pos_line[j].x + pos_line[j].x_off) && x_pos >= pos_line[j].x) {
								found = 1;
								break;
							}
						}

						/* If a better way is found, update the next target index and continue to loop. */
						if (found == 1) {
							printf("%s %d: found antoher suitable line, index: %d(%d)\n", __FUNCTION__, __LINE__, j, i);
							*x_next = x_tmp;
							*y_next = pos_line[j].y;
							level_next--;
							next_idx = j;
							level_set = 1;
							break;
						}

						printf("%s %d: %d %d %d %d %d\n", __FUNCTION__, __LINE__, g_pos_history[0].x, g_pos_history[0].y, g_pos_history[1].x,  g_pos_history[1].y, path_get_robot_direction());

						/* Try to avoid repeatly hit the obstcal ahead. */
						if (g_pos_history[0].x == g_pos_history[1].x && g_pos_history[0].y == g_pos_history[1].y) {
							/* Possibly ahead is blocked */
							printf("%s %d: level cur: %d\tlevel next: %d\n", __FUNCTION__, __LINE__, level_cur, level_next);
							if (level_cur - 1 == level_next + 1 && abs(*y_next - y_pos) <= 2) {

								/* Only handle the case of move towards NEG_Y & POS_Y. */
								if ((path_get_robot_direction() == NEG_Y && *y_next < y_pos) || (path_get_robot_direction() == POS_Y && *y_next > y_pos) ) {
									/* If the robot is repeatlly hitting the same obstcal, move back to the center of the line segment. */
									x1 = pos_line[cur_idx].x > pos_line[next_idx].x ? pos_line[cur_idx].x : pos_line[next_idx].x;
									x2 = pos_line[cur_idx].x + pos_line[cur_idx].x_off > pos_line[next_idx].x + pos_line[next_idx].x_off ? pos_line[next_idx].x + pos_line[next_idx].x_off : pos_line[cur_idx].x + pos_line[cur_idx].x_off;
									*x_next = (x1 + x2) / 2;
									*y_next = y_tmp;
									printf("%s %d: cur idx: %d\tnext idx: %d\tx1: %d\tx2: %d\tdest: (%d, %d)\n",
											__FUNCTION__, __LINE__, cur_idx, next_idx, x1, x2, *x_next, *y_next);
								}
							}
						}
						printf("%s %d: no further point to move, dest (%d,%d), last dir: %d\n", __FUNCTION__, __LINE__, *x_next, *y_next);
						lines_free();
						return 1;
					}
					/* Can move horizontally. */
					*x_next = x_tmp;
					*y_next = pos_line[i].y;
					level_next--;
					next_idx = i;
					level_set = 1;
				}
			} else {
				x1 = pos_line[i].x > pos_line[next_idx].x ? pos_line[i].x : pos_line[next_idx].x;
				x2 = pos_line[i].x + pos_line[i].x_off > pos_line[next_idx].x + pos_line[next_idx].x_off ? pos_line[next_idx].x + pos_line[next_idx].x_off : pos_line[i].x + pos_line[i].x_off;
				*x_next = (x1 + x2) / 2;
				*y_next = pos_line[i].y;
				printf("%s %d: %d %d\n", __FUNCTION__, __LINE__, *x_next, *y_next);
				lines_free();
				return 1;
			}
		}

		/* If no more node to trace back, report the target is not reachable. */
		if (level_set == 0) {
			printf("%s %d: no path to dest (%d, %d)\n", __FUNCTION__, __LINE__, x, y);
			*x_next = x_pos;
			*y_next = y_pos;
			lines_free();
			return -2;
		}
		//printf("%s %d: %d %d %d\n", __FUNCTION__, __LINE__, *x_next, *y_next, level_next);
	}
	printf("%s %d: next_idx: %d\tcur_idx: %d\tlevel_next: %d\tlevel_cur: %d\n", __FUNCTION__, __LINE__, next_idx, cur_idx, level_next, level_cur);

	printf("%s %d: %d %d\n", __FUNCTION__, __LINE__, *x_next, *y_next);
	lines_free();
	return 1;
}

/*
 * This becomes a dummy function, the process for finding the path is in path_move_to_unclean_area().
 *
 * @param x	The target X Coordinate that the robot wants to go
 * @param y	The target Y Coordinate that the robot wants to go
 * @param *x_next	The next X Coordinate that the robot should go before reaching the target
 * @param *y_next	The next Y Coordinate that the robot should go before reaching the target
 *
 * @return
 */
int16_t path_find_shortest_path(int16_t xID, int16_t yID, int16_t endx, int16_t endy, uint8_t bound)
{
	int16_t *x_next, *y_next;
	Cell_t	pos;

	bound = bound;
	pos.X = xID;
	pos.Y = yID;

	x_next = &xID;
	y_next = &yID;

	return path_move_to_unclean_area(pos, endx, endy, x_next, y_next);
}

#else

list <Cell_t> path_points;

/*
 * Give a target point, find the shorest path from the current robot position to the
 * target position. It use the grid map for shorest path searching, before searching,
 * it needs to reset the map, and then mark the obstcal into the shorest path map.
 * Start from the robot position, the cell value of it set to 1, then continuously
 * set the value of the map which cell is next to the cell value that just set in the
 * previous loop. Stop when there is not more value to set or the target position
 * can be reached.
 *
 * If the path is found, trace back from the target point to the current robot
 * position, and mark the cell value of the path as 6.
 *
 * @param xID	Robot X Coordinate
 * @param yID	Robot Y Coordinate
 * @param endx	The target X Coordinate
 * @param endy	The target Y Coordinate
 * @param bound	Limit to the search range to (xID, yID) and (endx, endy),
 * 		no path trace is needed when set. It is for checking a target
 * 		can be reach or not.
 * @param x_min The minimum range for X coordinate
 * @param x_max The maxmum range for X coordinate
 * @param y_min The minimum range for Y coordinate
 * @param y_max The maxmum range for Y coordinate
 *
 * @return	-2: Robot is trapped
 * 		-1: Path to target is not found
 * 		1:  Path to target is found
 * 		totalCost: Total cost
 *
 */
int16_t path_find_shortest_path_ranged(int16_t xID, int16_t yID, int16_t endx, int16_t endy, uint8_t bound, int16_t x_min, int16_t x_max, int16_t y_min, int16_t y_max) {
	uint16_t	next;
	int16_t	totalCost, costAtCell, targetCost, dest_dir;
	int16_t i, j, m, n, tracex, tracey, tracex_tmp, tracey_tmp, passValue, nextPassValue, passSet, offset;
	CellState cs;

	path_points.clear();

	/* Find the direction of target with reference to the current robot position. */
	if (xID == endx) {
		dest_dir = (yID > endy ? NEG_Y : POS_Y);
	} else if (yID == endy) {
		dest_dir = (xID > endx ? NEG_X : POS_X);
	} else if(abs(xID - endx) > abs(yID - endy)) {
		dest_dir = (xID > endx ? NEG_X : POS_X);
	} else {
		dest_dir = (yID > endy ? NEG_Y : POS_Y);
	}

	/* Reset the cells in the shorest path map. */
	for (i = x_min - 1; i <= x_max + 1; ++i) {
		for (j = y_min - 1; j <= y_max + 1; ++j) {
			Map_SetCell(SPMAP, (int32_t)i, (int32_t)j, COST_NO);
		}
	}

	/* Marked the obstcals to the shorest path map. */
	for (i = x_min - 1; i <= x_max + 1; ++i) {
		for (j = y_min - 1; j <= y_max + 1; ++j) {
			cs = Map_GetCell(MAP, i, j);
			if (cs >= BLOCKED && cs <= BLOCKED_BOUNDARY) {
				//for (m = ROBOT_RIGHT_OFFSET + 1; m <= ROBOT_LEFT_OFFSET - 1; m++) {
				for (m = ROBOT_RIGHT_OFFSET; m <= ROBOT_LEFT_OFFSET; m++) {
					for (n = ROBOT_RIGHT_OFFSET; n <= ROBOT_LEFT_OFFSET; n++) {
						Map_SetCell(SPMAP, (int32_t)(i + m), (int32_t)(j + n), COST_HIGH);
					}
				}
			}
		}
	}

	if (Map_GetCell(SPMAP, endx, endy) == COST_HIGH) {
		Map_SetCell(SPMAP, endx, endy, COST_NO);
	}

	/* Set the current robot position has the cost value of 1. */
	Map_SetCell(SPMAP, (int32_t)xID, (int32_t)yID, COST_1);

	/*
	 * Find the path to target from the current robot position. Set the cell values
	 * in shorest path map either 1, 2, 3, 4 or 5. This is a method like A-Star, starting
	 * from a start point, update the cells one level away, until we reach the target.
	 */
	offset = 0;
	passSet = 1;
	passValue = 1;
	nextPassValue = 2;
	while (Map_GetCell(SPMAP, endx, endy) == COST_NO && passSet == 1) {
		offset++;
		passSet = 0;

		/*
		 * The following 2 for loops is for optimise the computational time.
		 * Since there is not need to go through the whole map for seaching the
		 * cell that have the next pass value.
		 *
		 * It can use the offset to limit the range of searching, since in each loop
		 * the cell (X -/+ offset, Y -/+ offset) would be set only. The cells far away
		 * to the robot position won't be set.
		 */
		for (i = xID - offset; i <= xID + offset; i++) {
			if (i < x_min || i > x_max)
				continue;

			for (j = yID - offset; j <= yID + offset; j++) {
				if (j < y_min || j > y_max)
					continue;

				/* Found a cell that has a pass value equal to the current pass value. */
				if(Map_GetCell(SPMAP, i, j) == passValue) {
					/* Set the lower cell of the cell which has the pass value equal to current pass value. */
					if (Map_GetCell(SPMAP, i - 1, j) == COST_NO) {
						Map_SetCell(SPMAP, (int32_t)(i - 1), (int32_t)j, (CellState)nextPassValue);
						passSet = 1;
					}

					/* Set the upper cell of the cell which has the pass value equal to current pass value. */
					if (Map_GetCell(SPMAP, i + 1, j) == COST_NO) {
						Map_SetCell(SPMAP, (int32_t)(i + 1), (int32_t)j, (CellState)nextPassValue);
						passSet = 1;
					}

					/* Set the cell on the right hand side of the cell which has the pass value equal to current pass value. */
					if (Map_GetCell(SPMAP, i, j - 1) == COST_NO) {
						Map_SetCell(SPMAP, (int32_t)i, (int32_t)(j - 1), (CellState)nextPassValue);
						passSet = 1;
					}

					/* Set the cell on the left hand side of the cell which has the pass value equal to current pass value. */
					if (Map_GetCell(SPMAP, i, j + 1) == COST_NO) {
						Map_SetCell(SPMAP, (int32_t)i, (int32_t)(j + 1), (CellState)nextPassValue);
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
	if (Map_GetCell(SPMAP, endx, endy) == COST_NO) {
		ROS_WARN("target point (%d, %d) is not reachable(0), return -2.", endx, endy);
#ifdef	DEBUG_SM_MAP
		debug_map(SPMAP, endx, endy);
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
	t.X = tracex = tracex_tmp = endx;
	t.Y = tracey = tracey_tmp = endy;
	path_points.push_back(t);

	next = 0;
	dest_dir = (path_get_robot_direction() == POS_Y || path_get_robot_direction() == NEG_Y) ? 1: 0;
	ROS_INFO("%s %d: dest dir: %d", __FUNCTION__, __LINE__, dest_dir);
	while (tracex != xID || tracey != yID) {
		costAtCell = Map_GetCell(SPMAP, tracex, tracey);
		targetCost = costAtCell - 1;

		/* Reset target cost to 5, since cost only set from 1 to 5 in the shorest path map. */
		if (targetCost == 0)
			targetCost = COST_5;

		/* Set the cell value to 6 if the cells is on the path. */
		Map_SetCell(SPMAP, (int32_t)tracex, (int32_t)tracey, COST_PATH);

#define COST_SOUTH	{											\
				if (next == 0 && (Map_GetCell(SPMAP, tracex - 1, tracey) == targetCost)) {	\
					tracex--;								\
					next = 1;								\
					dest_dir = 1;								\
				}										\
			}

#define COST_WEST	{											\
				if (next == 0 && (Map_GetCell(SPMAP, tracex, tracey - 1) == targetCost)) {	\
					tracey--;								\
					next = 1;								\
					dest_dir = 0;								\
				}										\
			}

#define COST_EAST	{											\
				if (next == 0 && (Map_GetCell(SPMAP, tracex, tracey + 1) == targetCost)) {	\
					tracey++;								\
					next = 1;								\
					dest_dir = 0;								\
				}										\
			}

#define COST_NORTH	{											\
				if (next == 0 && Map_GetCell(SPMAP, tracex + 1, tracey) == targetCost) {	\
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
		if (path_points.back().X != tracex && path_points.back().Y != tracey) {
			t.X = tracex_tmp;
			t.Y = tracey_tmp;
			path_points.push_back(t);
		}
		tracex_tmp = tracex;
		tracey_tmp = tracey;
	}
	Map_SetCell(SPMAP, (int32_t)tracex, (int32_t)tracey, COST_PATH);

	t.X = tracex_tmp;
	t.Y = tracey_tmp;
	path_points.push_back(t);

	path_display_path_points();

	return totalCost;
}

/*
 * Give a target point, find the shorest path from the current robot position to the
 * target position.
 *
 * @param xID	Robot X Coordinate
 * @param yID	Robot Y Coordinate
 * @param endx	Target X Coordinate
 * @param endy	Target Y Coordinate
 * @param bound	Limit to the search range to (xID, yID) and (endx, endy)
 *
 * @return	-2: Robot is trapped
 * 		-1: Path to target is not found
 * 		1:  Path to target is found
 * 		(totalCost: from function path_find_shortest_path_ranged)
 *
 */
int16_t path_find_shortest_path(int16_t xID, int16_t yID, int16_t endx, int16_t endy, uint8_t bound) {
	int16_t val;
	int16_t x_min, x_max, y_min, y_max;

	if (bound == 1) {
		/* If bound is set, set the search range. */
		x_min = (xID > endx ? endx : xID) - 8;
		x_max = (xID > endx ? xID : endx) + 8;
		y_min = (yID > endy ? endy : yID) - 8;
		y_max = (yID > endy ? yID : endy) + 8;
		ROS_INFO("shortest path(%d): endx: %d\tendy: %d\tx: %d - %d\ty: %d - %d", __LINE__, endx, endy, x_min, x_max, y_min, y_max);
		val =  path_find_shortest_path_ranged(xID, yID, endx, endy, bound, x_min, x_max, y_min, y_max);
	} else {
		/* If bound is not set, set the search range to the whole map. */
		path_get_range(&x_min, &x_max, &y_min, &y_max);
		val =  path_find_shortest_path_ranged(xID, yID, endx, endy, bound, x_min, x_max, y_min, y_max);
		ROS_INFO("shortest path(%d): endx: %d\tendy: %d\tx: %d - %d\ty: %d - %d\t return: %d", __LINE__, endx, endy, x_min, x_max, y_min, y_max, val);
	}

	return val;
}

/*
 * Give a target point, find the shorest path from the current robot position to the
 * target position.
 *
 * @param xID	Robot X Coordinate
 * @param yID	Robot Y Coordinate
 * @param endx	Target X Coordinate
 * @param endy	Target Y Coordinate
 * @param bound	Limit to the search range to (xID, yID) and (endx, endy)
 *
 * @return	-2: Robot is trapped
 * 		-1: Path to target is not found
 * 		1:  Path to target is found
 * 		(totalCost: from function path_find_shortest_path_ranged)
 *
 */
int16_t WF_path_find_shortest_path(int16_t xID, int16_t yID, int16_t endx, int16_t endy, uint8_t bound) {
	int16_t val;
	int16_t x_min, x_max, y_min, y_max;

	if (bound == 1) {
		/* If bound is set, set the search range. */
		x_min = (xID > endx ? endx : xID) - 8;
		x_max = (xID > endx ? xID : endx) + 8;
		y_min = (yID > endy ? endy : yID) - 8;
		y_max = (yID > endy ? yID : endy) + 8;
		ROS_INFO("shortest path(%d): endx: %d\tendy: %d\tx: %d - %d\ty: %d - %d\n", __LINE__, endx, endy, x_min, x_max, y_min, y_max);
		val =  path_find_shortest_path_ranged(xID, yID, endx, endy, bound, x_min, x_max, y_min, y_max);
	} else {
		/* If bound is not set, set the search range to the whole map. */
		path_get_range(&x_min, &x_max, &y_min, &y_max);
		x_min = x_min - 8;
		x_max = x_max + 8;
		y_min = y_min - 8;
		y_max = y_max + 8;

		val =  path_find_shortest_path_ranged(xID, yID, endx, endy, bound, x_min, x_max, y_min, y_max);
		ROS_INFO("shortest path(%d): endx: %d\tendy: %d\tx: %d - %d\ty: %d - %d\t return: %d\n", __LINE__, endx, endy, x_min, x_max, y_min, y_max, val);
	}

	debug_map(SPMAP, endx, endy);
	return val;
}

/*
 * By given a target, find the shortest path to the target. When finding the shorest path,
 * a grid map is used, and starting form the target position, this function will trace back
 * the path of which values are marked as 6 in the shorest path map. It will return the
 * coordinate by pointer to the caller function. When the g_direct_go flag is set, it will
 * enable the robot to move directly to the next target point without limited to turn
 * 90, 180 or 270 degree, but it only happens when there is no obstcal in between the current
 * robot position and the next target point. It the g_direct_go flag is not set, the robot
 * is limited to turn 90, 180 and/or 270 degree, then go to the target.
 *
 * @param pos	The current robot position
 * @param x	The target X Coordinate that the robot wants to go
 * @param y	The target Y Coordinate that the robot wants to go
 * @param *x_next	The next X Coordinate that the robot should go before reaching the target
 * @param *y_next	The next Y Coordinate that the robot should go before reaching the target
 *
 * @return	-2: Robot is trapped
 * 		-1: Path to target is not found
 * 		1:  Path to target is found
 * 		(totalCost: from function path_find_shortest_path)
 */
int16_t path_move_to_unclean_area(Cell_t position, int16_t x, int16_t y, int16_t *x_next, int16_t *y_next) {
	int16_t	retval;
	uint8_t	blocked, stage;
	int16_t	i, j, ei, ej, si, sj, x_path, y_path, offset = 0;

	Cell_t pos;

	path_reset_path_points();

	/* Find the shortest path to the target by using shorest path grid map. */
	retval = path_find_shortest_path(position.X, position.Y, x, y, 0);
	if (retval < 0)
		return retval;

	/* g_direct_go flag is enabled. */
	if (g_direct_go == 1) {
		x_path = pos.X;
		y_path = pos.Y;

		/*
		 * Trace back the path from the current robot position by using the shorest path
		 * grid map, trace the cells that have the value of 6. Check whether there is obstcal
		 * in the way to the target, if obstcal is found, return the point that just before
		 * finding the obstcal as the next point to move.
		 */
		while (x_path != x || y_path != y) {
			/*
			 * Starting from the current robot position, finding the cells that marked as 6
			 * in the shorest path grid map.
			 */
			stage = 0;
			if (Map_GetCell(SPMAP, x_path - 1, y_path) == COST_PATH) {
				x_path--;
				stage = 1;
				Map_SetCell(SPMAP, (int32_t)(x_path), (int32_t)(y_path), COST_NO);
			} else if (Map_GetCell(SPMAP, x_path, y_path + 1) == COST_PATH) {
				y_path++;
				stage = 2;
				Map_SetCell(SPMAP, (int32_t)(x_path), (int32_t)(y_path), COST_NO);
			} else if (Map_GetCell(SPMAP, x_path + 1, y_path) == COST_PATH) {
				x_path++;
				stage = 3;
				Map_SetCell(SPMAP, (int32_t)(x_path), (int32_t)(y_path), COST_NO);
			} else if (Map_GetCell(SPMAP, x_path, y_path - 1) == COST_PATH) {
				y_path--;
				stage = 4;
				Map_SetCell(SPMAP, (int32_t)(x_path), (int32_t)(y_path), COST_NO);
			}

			if (stage == 0)
				break;

			/* Check for the obstcal from current robot position and the new next point to move. */
			si = pos.X < x_path ? pos.X : x_path;
			ei = pos.X < x_path ? x_path : pos.X;
			sj = pos.Y < y_path ? pos.Y : y_path;
			ej = pos.Y < y_path ? y_path : pos.Y;

			blocked = 0;
			for (i = si; i <= ei && blocked == 0; i++) {
				for (j = sj; j <= ej && blocked == 0; j++) {
					if(Map_GetCell(SPMAP, i, j) == COST_HIGH) {
						blocked = 1;
					}
				}
			}

			/* If obstcal is found, stop the loop and return the next point to move. */
			if (blocked == 1) {
				if (stage == 1) {
					x_path++;
				} else if (stage == 2) {
					y_path--;
				} else if (stage == 3) {
					x_path--;
				} else if (stage == 4) {
					y_path++;
				}
				break;
			}
		}
		*x_next = x_path;
		*y_next = y_path;
	} else {
		*x_next = x;
		*y_next = y;

#ifdef	PP_MOVE_TO_MIDDLE_OF_PATH
		if (path_points.size() > 3) {
			list<Cell_t>::iterator it = path_points.begin();
			for (i = 0; i < path_points.size() - 3; i++) {
				list<Cell_t>::iterator it_ptr1 = it;

				list<Cell_t>::iterator it_ptr2 = it_ptr1;
				it_ptr2++;

				list<Cell_t>::iterator it_ptr3 = it_ptr2;
				it_ptr3++;

				bool blocked_min, blocked_max;
				blocked_min = blocked_max = false;
				if (it_ptr2->X == it_ptr3->X) {		// X coordinates are the same for p1, p2, find a better Y coordinate.
					int16_t x_min, x_max;
					x_min = x_max = it_ptr2->X;

					sj = it_ptr1->X > it_ptr2->X ? it_ptr2->X : it_ptr1->X;
					ej = it_ptr1->X > it_ptr2->X ? it_ptr1->X : it_ptr2->X;
					si = it_ptr2->Y > it_ptr3->Y ? it_ptr3->Y : it_ptr2->Y;
					ei = it_ptr2->Y > it_ptr3->Y ? it_ptr2->Y : it_ptr3->Y;

					while (blocked_min == false || blocked_max == false) {
						for (j = sj; j <= ej && (blocked_min == false || blocked_max == false); j++) {
							for (i = si; i <= ei && (blocked_min == false || blocked_max == false); i++) {
								if (blocked_min == false && (x_min - 1 < sj || Map_GetCell(SPMAP, x_min - 1, i) == COST_HIGH)) {
									blocked_min = true;
								}
								if (blocked_max == false && (x_max + 1 > ej || Map_GetCell(SPMAP, x_max + 1, i) == COST_HIGH)) {
									blocked_max = true;
								}
							}
						}
						if (blocked_min == false) {
							x_min--;
						}
						if (blocked_max == false) {
							x_max++;
						}
					}

					ROS_INFO("%s %d: x_min: %d\tx_max: %d\n", __FUNCTION__, __LINE__, x_min, x_max);
					if (x != (x_min + x_max) / 2) {
						it_ptr2->X = it_ptr3->X = (x_min + x_max) / 2;
					}
				} else {
					int16_t y_min, y_max;
					y_min = y_max = it_ptr2->Y;

					sj = it_ptr1->Y > it_ptr2->Y ? it_ptr2->Y : it_ptr1->Y;
					ej = it_ptr1->Y > it_ptr2->Y ? it_ptr1->Y : it_ptr2->Y;
					si = it_ptr2->X > it_ptr3->X ? it_ptr3->X : it_ptr2->X;
					ei = it_ptr2->X > it_ptr3->X ? it_ptr2->X : it_ptr3->X;
					while (blocked_min == false || blocked_max == false) {
						for (j = sj; j <= ej && (blocked_min == false || blocked_max == false); j++) {
							for (i = si; i <= ei && (blocked_min == false || blocked_max == false); i++) {
								if (blocked_min == false && (y_min - 1 < sj || Map_GetCell(SPMAP, i, y_min - 1) == COST_HIGH)) {
									blocked_min = true;
								}
								if (blocked_max == false && (y_max + 1 > ej || Map_GetCell(SPMAP, i, y_max + 1) == COST_HIGH)) {
									blocked_max = true;
								}
							}
						}
						if (blocked_min == false) {
							y_min--;
						}
						if (blocked_max == false) {
							y_max++;
						}
					}

					ROS_INFO("%s %d: y_min: %d\ty_max: %d\n", __FUNCTION__, __LINE__, y_min, y_max);
					if (y != (y_min + y_max) / 2) {
						it_ptr2->Y = it_ptr3->Y = (y_min + y_max) / 2;
					}
				}

				it++;
			}
		}

		path_display_path_points();

#endif

		if (path_points.size() > 1) {
			i = 0;
			for (list<Cell_t>::iterator it = path_points.begin(); it != path_points.end() && i <= 1; ++it, ++i) {
				if (i != 1) {
					continue;
				} else {
					*x_next = it->X;
					*y_next = it->Y;
				}
			}
		}
	}

	retval = 1;
	return retval;
}

int path_get_path_points_count()
{
	return path_points.size();
}

list<Cell_t> *path_get_path_points()
{
	return &path_points;
}

void path_reset_path_points()
{
	path_points.clear();
}

void path_display_path_points()
{
	std::string     msg = __FUNCTION__;

	msg += " " + std::to_string(__LINE__) + ": ";
	for (list<Cell_t>::iterator it = path_points.begin(); it != path_points.end(); ++it) {
		msg += "(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ")->";
	}
	msg += "\n";
	ROS_INFO(msg.c_str());
}

#endif
