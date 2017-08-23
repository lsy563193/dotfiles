#ifndef _ROBOTBASE_H_
#define _ROBOTBASE_H_

#include <sys/types.h>
#include <ros/ros.h>
#include <pp/slam_angle_offset.h>
#include "config.h"

#if __ROBOT_X900
#define RECEI_LEN 57
extern uint8_t receiStream[RECEI_LEN];
#define SEND_LEN 21
#elif __ROBOT_X400
#define RECEI_LEN	50
extern uint8_t receiStream[RECEI_LEN];
#define SEND_LEN 19
#endif
//for tilt detct
#ifndef TILT_COUNT_REACH
#define TILT_COUNT_REACH (20)
#endif

#define DIF_TILT_X_VAL 70
#define DIF_TILT_Y_VAL 70
#define DIF_TILT_Z_VAL 40

extern bool g_is_tilt;

extern uint8_t sendStream[SEND_LEN] ;
extern bool robotbase_beep_update_flag;
extern int robotbase_speaker_sound_loop_count;
extern uint8_t robotbase_sound_code;
extern int robotbase_speaker_sound_time_count;
extern int robotbase_speaker_silence_time_count;
// For led control.
extern uint8_t robotbase_led_type;
extern bool robotbase_led_update_flag;
extern uint8_t robotbase_led_color;
extern uint16_t robotbase_led_cnt_for_switch;
extern uint16_t live_led_cnt_for_switch;

extern bool key_or_clean_button_detected;
extern int OBS_adjust_count;
int robotbase_init();
void robotbase_deinit(void);
void robotbase_reset_send_stream(void);
bool is_robotbase_stop(void);
void *serial_receive_routine(void*);
void *robotbase_routine(void*);
void *serial_send_routine(void*);
void process_beep();
void process_led();
void robotbase_reset_odom_pose(void);
void robotbase_restore_slam_correction(void);
void robotbase_obs_adjust_count(int count);
bool is_turn(void);
void set_acc_init_data();
#endif
