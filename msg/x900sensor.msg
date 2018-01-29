# left_wheel_speed: Left_wheel speed in m/s.
float32 left_wheel_speed
# right_wheel_speed: Left_wheel speed in m/s.
float32 right_wheel_speed

# left_wheel_cliff: Left wheel cliff switch status.
bool left_wheel_cliff
# right_wheel_cliff: Right wheel cliff switch status.
bool right_wheel_cliff

# gyro_calibration: Gyro calibration status.
bool gyro_calibration

# angle: Gyro angle in degree.
float32 angle

# angle_v: Gyro angle velocity in degree/s.
float32 angle_v

# x_acc/y_acc/z_acc: Gyro acceleration value for x/y/z in mG.
int16 x_acc
int16 y_acc
int16 z_acc

# left_wall: Left wall sensor value.
int16 left_wall
# right_wall: Right wall sensor value.
int16 right_wall

# left_obs: Left obs sensor value.
int16 left_obs
# front_obs: Front obs sensor value.
int16 front_obs
# right_obs: Right obs sensor value.
int16 right_obs

# left_bumper: Left bumper status.
bool left_bumper
# right_bumper: Right bumper status.
bool right_bumper
# lidar_bumper: Lidar bumper status.
bool lidar_bumper

# left_cliff: Left cliff status.
bool left_cliff
# right_cliff: Right cliff status.
bool right_cliff
# front_cliff: Front cliff status.
bool front_cliff

# remote: Remote receive signal.
# 0: not received
# 1: spot
# 2: follow wall
# 3: home
# 4: clean
# 5: max
# 6: right
# 7: left
# 8: forward
uint8 remote

# rcon: Rcon receiving charger signal.
uint32 rcon

# virtual_wall: Rcon receiving virtual wall signal.
uint16 virtual_wall

# key: Robot key status.
bool key

# plan: Plan status.
# 0: Nothing happens
# 1: Received a new appointment
# 2: Cancel a appointment
# 3: Start cleaning as planned
# 4: New appointment activated
uint8 plan

# charge_status: Charge status.
# 0: Not on charger stub.
# 1: On charger stub and charging.
# 2: On charger stub but not charging.
# 3: Direct connect to charge line but not charging.
# 4: Direct connect to charge line and charging.
uint8 charge_status

# main_board_sleep_status: Robot STM32 main board sleep status.
bool main_board_sleep_status

# water_tank: Water tank status.
bool water_tank

# battery: Battery voltage in V.
float32 battery

# vacuum_oc: Vacuum over current status.
bool vacuum_oc

# right_brush_oc: Right brush over current status.
bool right_brush_oc

# left_brush_oc: Left brush over current status.
bool left_brush_oc

# main_brush_oc: Main brush over current status.
bool main_brush_oc

# left_wheel_oc: Left wheel over current status.
bool left_wheel_oc

# right_wheel_oc: Right wheel over current status.
bool right_wheel_oc
