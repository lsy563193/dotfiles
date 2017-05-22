#lw_vel,rw_vel:left_wheel and right_wheel volcity in m/s
float32 lw_vel
float32 rw_vel

# angel and angle velocity in degress/s
float32 angle
float32 angle_v

#left and right wheel current in mA
float32 lw_crt
float32 rw_crt

#infrated detect value,0~65535
int16 left_wall
int16 right_wall
#left right front obs value,0~65535
int16 l_obs
int16 f_obs
int16 r_obs

# bumper value:1 hit,0 not hit
bool rbumper
bool lbumper

# ir controler signal
uint8 ir_ctrl

#c_stub: charge stub signal,bit7 forward,6 left,5 right,4 max,3 clean,2 go home,1 random,0 spot
uint32 c_stub

# on board key ,value:0 no press, 1 press
uint8 key

#c_s: charge status, 0: Not on charger stub, 1: On charger stub and charging, 2: On charger stub but not charging.
uint8 c_s

#w_tank: water tank status,1:set,0:not set
bool w_tank

#batv: battery voltage,in xx.xx v
uint16 batv

#right left and front cliff value,0~65535
int16 rcliff
int16 lcliff
int16 fcliff

# left,right, main brush and vccum over current value,1:over current, 0:no over current
bool lbrush_oc
bool mbrush_oc
bool rbrush_oc
bool vcum_oc

# Acceleration for x, y, z
float32 x_acc
float32 y_acc
float32 z_acc

# omni-wheel 0--10000
int16 omni_wheel
#visual wall 
int16 visual_wall
#gyroscope dynamic ragulate
int8 gyro_dymc

#plan, true: robot got a plan, false: nothing had happened
bool plan

#vacuum self check status
#0x00:not in checking mode / check finished and no error
#0x10:checking
#0x20:check finished and vacuum error
uint8 vacuum_selfcheck_status
