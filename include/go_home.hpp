#include <unistd.h>
#include <stdint.h>

#define ROUND_LEFT			0x01
#define ROUND_RIGHT			0x02

void go_home(void);
void go_to_charger(void);
bool go_home_check_move_back_finish(uint8_t type);
bool go_home_check_turn_finish(int16_t target_angle);
void go_home_register_events(void);
void go_home_unregister_events(void);
void go_home_handle_key_clean(bool, bool);
void go_home_handle_charge_detect(bool, bool);
void go_home_handle_remote_clean(bool, bool);
void go_home_handle_cliff_all(bool, bool);
void go_home_handle_bumper(bool, bool);
void go_home_handle_cliff(bool, bool);
void go_home_handle_battery_low(bool, bool);
void go_home_handle_over_current_brush_main(bool, bool);
void go_home_handle_over_current_wheel_left(bool, bool);
void go_home_handle_over_current_wheel_right(bool, bool);
void go_home_handle_over_current_suction(bool, bool);
bool turn_connect(void);
