//
// Created by lsy563193 on 7/7/17.
//

#ifndef PP_MOVETYPE_H
#define PP_MOVETYPE_H

#include "mathematics.h"
#include "path_planning.h"

typedef enum {
	MT_LINEARMOVE = 0,
	MT_FOLLOW_LEFT_WALL,
	MT_FOLLOW_RIGHT_WALL,
	MT_GO_TO_CHARGER,
} MoveType;

bool mt_is_right();

bool mt_is_left();

bool mt_is_follow_wall();

bool mt_is_linear();

bool mt_is_go_to_charger();

MoveType mt_get();

void mt_set(MoveType mt);

void mt_update(const Cell_t& curr, PPTargetType& path);

/*
 * param:	dir: The direction of last movement, actually it is for direction of linear movement.
 * param:	curr: Current robot position.
 * param:	path: The path to target cell.
 *
 * return:	true: Robot should switch move type to following wall.
 * 			false: Robot should not switch move type to following wall.
 */
bool mt_should_follow_wall(const int16_t dir, const Cell_t& curr, PPTargetType& path);

#endif //PP_MOVETYPE_H
