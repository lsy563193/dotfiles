#ifndef _ROBOTBASE_H_
#define _ROBOTBASE_H_

#include <sys/types.h>
#include <ros/ros.h>
extern bool robotbase_thread_stop;
extern bool is_robotbase_init;
extern uint8_t receiStream[41];

void robotbase_init(void);
void robotbase_deinit(void);
void* serial_receive(void*);
void* baserun(void*);
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
#endif
