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

void mt_update(const Cell_t& curr, PPTargetType& path);

void mt_set(MoveType mt);

#endif //PP_MOVETYPE_H
