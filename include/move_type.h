//
// Created by lsy563193 on 7/7/17.
//

#ifndef PP_MOVETYPE_H
#define PP_MOVETYPE_H

#include "mathematics.h"
#include "path_planning.h"

const int MT_LINEARMOVE = 0;
const int MT_FOLLOW_LEFT_WALL =1 ;
const int MT_FOLLOW_RIGHT_WALL = 2;
const int MT_GO_TO_CHARGER = 3;

class MoveType {
public:
	MoveType() { mt_ = MT_LINEARMOVE; }

	bool is_right();

	bool is_left();

	bool is_follow_wall();

	bool is_linear();

	bool is_go_to_charger();

	int get();

	void set(int mt);

	void update(const Cell_t &curr, PPTargetType &path);

/*
 * param:	dir: The direction of last movement, actually it is for direction of linear movement.
 * param:	curr: Current robot position.
 * param:	path: The path to target cell.
 *
 * return:	true: Robot should switch move type to following wall.
 * 			false: Robot should not switch move type to following wall.
 */
	bool should_follow_wall(const MapDirection dir, const Cell_t &curr, PPTargetType &path);

private:
	int mt_;
};
extern MoveType mt;
#endif //PP_MOVETYPE_H
