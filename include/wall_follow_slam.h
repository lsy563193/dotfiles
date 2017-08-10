#ifndef __WALL_FOLLOW_SLAM_H__
#define __WALL_FOLLOW_SLAM_H__

#include "config.h"

#include "mathematics.h"
#include "debug.h"
#include <functional>
#include <future>
#include "wall_follow_trapped.h"

uint8_t wf_clear(void);
void wf_update_map();
bool wf_is_end();
uint8_t wf_break_wall_follow(void);
bool wf_is_go_home();
bool wf_is_first();
bool wf_is_reach_start();
#endif
