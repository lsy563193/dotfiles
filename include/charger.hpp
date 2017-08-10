#include <unistd.h>
#include <stdint.h>

void charge_function();
void charge_register_event(void);
void charge_unregister_event(void);
void charge_handle_charge_detect(bool state_now, bool state_last);
void charge_handle_remote_plan(bool state_now, bool state_last);
void charge_handle_key_clean(bool state_now, bool state_last);
void charge_handle_cliff_all(bool state_now, bool state_last);
void charge_handle_remote_cleaning(bool stat_now, bool state_last);
bool charge_turn_connect(void);
