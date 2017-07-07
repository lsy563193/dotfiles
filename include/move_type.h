//
// Created by lsy563193 on 7/7/17.
//

#ifndef PP_MOVETYPE_H
#define PP_MOVETYPE_H


typedef enum {
	CM_LINEARMOVE = 0,
	CM_CURVEMOVE,
	CM_FOLLOW_LEFT_WALL,
	CM_FOLLOW_RIGHT_WALL,
} CMMoveType;

extern CMMoveType g_cm_move_type;

bool mt_is_fallwall();

bool mt_update(Point32_t *next_point, Point32_t target_point, uint16_t dir);

/*
class MoveType {

};
*/


#endif //PP_MOVETYPE_H
