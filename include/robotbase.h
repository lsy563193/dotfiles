#ifndef _ROBOTBASE_H_
#define _ROBOTBASE_H_

#include <sys/types.h>
#include <ros/ros.h>
#include <pp/slam_angle_offset.h>
#define RECEI_LEN	50
#define SEND_LEN 19
extern uint8_t receiStream[RECEI_LEN];
extern uint8_t sendStream[SEND_LEN] ;
int robotbase_init();
void robotbase_deinit(void);
bool is_robotbase_stop(void);
void *serial_receive_runtime(void*);
void *robotbase_runtime(void*);
void *serial_send_runtime(void*);
void slam_angle_offset_callback(const pp::slam_angle_offset::ConstPtr& msg);

#endif
