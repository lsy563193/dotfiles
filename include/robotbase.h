#ifndef _ROBOTBASE_H_
#define _ROBOTBASE_H_

#include <sys/types.h>
#include <ros/ros.h>

#include "config.h"

#define	CTL_WHEEL_LEFT_HIGH 2
#define	CTL_WHEEL_LEFT_LOW  3
#define	CTL_WHEEL_RIGHT_HIGH  4
#define	CTL_WHEEL_RIGHT_LOW 5
#define	CTL_VACCUM_PWR 6
#define	CTL_BRUSH_LEFT 7
#define	CTL_BRUSH_RIGHT 8
#define	CTL_BRUSH_MAIN 9
#define	CTL_BUZZER 10
#define	CTL_MAIN_PWR 11
#define	CTL_CHARGER 12
#define	CTL_LED_RED 13
#define	CTL_LED_GREEN 14

#define REC_LW_S_H 2//left wheel speed high byte 
#define REC_LW_S_L 3
#define REC_RW_S_H 4//right wheel speed high byte
#define REC_RW_S_L 5

#define REC_ANGLE_H 6 //angle high byte
#define REC_ANGLE_L 7
#define REC_ANGLE_V_H 8//angle velocity high byte
#define REC_ANGLE_V_L 9

#define REC_LW_C_H 10//left wheel current high byte
#define REC_LW_C_L 11
#define REC_RW_C_H 12
#define REC_RW_C_L 13

#define REC_L_WALL_H 14 //left wall sensor high byte
#define REC_L_WALL_L 15

#define REC_L_OBS_H 16 //left obs high byte
#define REC_L_OBS_L 17
#define REC_F_OBS_H 18//front obs high byte
#define REC_F_OBS_L 19
#define REC_R_OBS_H 20//right obs high byte
#define REC_R_OBS_L 21

#define REC_R_WALL_H 22//right wall sensor high byte
#define REC_R_WALL_L 23//low byte

#define REC_BUMPER 24//bumper 

#define REC_REMOTE_IR 25//remote ir control

#define REC_CHARGE_STUB_4 26//charge stub signal byte 4
#define REC_CHARGE_STUB_3 27//byte 3
#define REC_CHARGE_STUB_2 28//byte 2
#define REC_CHARGE_STUB_1 29//byte 1

#define REC_VISUAL_WALL_H 30//visual wall high byte
#define REC_VISUAL_WALL_L 31

#define REC_KEY 32//key on robot 

#define REC_CHARGE_STATE 33

#define REC_WATER_TANK 34

#define REC_BAT_V 35//battery voltage

#define REC_L_CLIFF_H 36//left cliff high byte
#define REC_L_CLIFF_L 37
#define REC_F_CLIFF_H 38//front cliff high byte
#define REC_F_CLIFF_L 39
#define REC_R_CLIFF_H 40//right cliff high byte
#define REC_R_CLIFF_L 41

#define REC_CL_OC 42//clean tools over current

#define REC_GYRO_DYMC 43//gyro dynamic

#define REC_OMNI_W_H 44//omni wheel count value high byte
#define REC_OMNI_W_L 45

#define REC_XACC_H 46//x acceleration high byte
#define REC_XACC_L 47
#define REC_YACC_H 48
#define REC_YACC_L 49
#define REC_ZACC_H 50
#define REC_ZACC_L 51

#define REC_PLAN 52//set clean plan

#if __ROBOT_X400

#define	CTL_GYRO 15
#define	CTL_CRC 16

#define RECEI_LEN	50
extern uint8_t g_receive_stream[RECEI_LEN];
#define SEND_LEN 19

#elif __ROBOT_X900

#define CTL_OMNI_RESET 15
#define CTL_GYRO 16
#define CTL_CMD				17
#define CTL_CRC				18

#define RECEI_LEN 57
extern uint8_t g_receive_stream[RECEI_LEN];
#define SEND_LEN 21

#endif

extern uint8_t g_send_stream[SEND_LEN];
extern boost::mutex g_send_stream_mutex;

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
#endif
