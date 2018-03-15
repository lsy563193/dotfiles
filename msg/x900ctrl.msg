# Two bytes for controlling left wheel.
int16 left_wheel_speed

# Two bytes for controlling right wheel.
int16 right_wheel_speed

# One byte for controlling vacuum PWM.
uint8 vacuum_PWM

# One byte for controlling left brush PWM.
uint8 left_brush_PWM

# One byte for controlling right brush PWM.
uint8 right_brush_PWM

# One byte for controlling main brush PWM.
uint8 main_brush_PWM

# One byte for controlling beeper.
uint8 beeper_sound_code

# One byte for sending main board mode.
uint8 main_board_mode

# One byte for controlling charge status.
uint8 charge_control

# One byte for controlling red led.
uint8 led_red_brightness

# One byte for controlling green led.
uint8 led_green_brightness

# One byte for mix command.
# bit 0 for wifi led controlling.
# bit 1 for vacuum exception resume control.
# bit 2 for switch of dynamic adjustment.
# bit 3 for switch of gyro.
# bit 4-7 reserved.
uint8 wifi_led
uint8 vacuum_exception_ctrl
uint8 gyro_dynamic_ctrl
uint8 gyro_switch

# one byte for water tank.
# bit 0-6 for swing motor PWM.
# bit 7 for pump switch.
uint8 swing_motor_pwm
bool pump_switch

# One byte for key validation.
uint8 key_validation

# One byte for crc checking.
uint8 crc
