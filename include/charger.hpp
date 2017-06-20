#include <unistd.h>
#include <stdint.h>

#define Round_Left			0x01
#define Round_Right			0x02

void Charge_Function();
void GoHome(void);
void Around_ChargerStation(uint8_t Dir);
uint8_t Check_Position(uint8_t Dir);
void By_Path(void);
uint8_t Home_Check_Current(void);
void Home_Motor_Set(void);
void charge_register_event(void);
void charge_unregister_event(void);
void charge_handle_charge_detect(bool state_now, bool state_last);
void charge_handle_remote_plan(bool state_now, bool state_last);
void charge_handle_key_clean(bool state_now, bool state_last);
void charge_handle_remote_cleaning(bool stat_now, bool state_last);
