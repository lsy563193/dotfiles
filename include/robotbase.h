#ifndef _ROBOTBASE_H_
#define _ROBOTBASE_H_

#include <sys/types.h>
#include <ros/ros.h>
#include <pp/peripheral.h>
#include <pp/slam_angle_offset.h>

#include "control.h"

int robotbase_init();
void robotbase_deinit(void);
bool is_robotbase_stop(void);
void *serial_receive(void*);
void *base_run(void*);
void slam_angle_offset_callback(const pp::slam_angle_offset::ConstPtr& msg);

#endif
