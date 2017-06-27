#include <unistd.h>
#include <stdint.h>

#define ROUND_LEFT			0x01
#define ROUND_RIGHT			0x02

void go_home(void);
void around_chargerstation(uint8_t Dir);
uint8_t check_position(uint8_t Dir);
void by_path(void);
uint8_t home_check_current(void);
void home_motor_set(void);
void go_home_register_events(void);
void go_home_unregister_events(void);
void go_home_handle_key_clean(bool, bool);
void go_home_handle_charge_detect(bool, bool);
void go_home_handle_remote_clean(bool, bool);
void go_home_handle_cliff_all(bool, bool);
void go_home_handle_bumper_left(bool, bool);
void go_home_handle_bumper_right(bool, bool);
void go_home_handle_bumper_all(bool, bool);
void go_home_handle_cliff(bool, bool);
void go_home_handle_battery_low(bool, bool);
uint8_t turn_connect(void);
