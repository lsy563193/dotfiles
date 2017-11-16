#include <unistd.h>
#include <stdint.h>
#include <event_manager.h>

class Charge_EventHandle:public EventHandle {
	void charge_detect(bool state_now, bool state_last);

	void remote_plan(bool state_now, bool state_last);

	void key_clean(bool state_now, bool state_last);

	void cliff_all(bool state_now, bool state_last);

	void remote_clean(bool stat_now, bool state_last);

	void remote_home(bool stat_now, bool state_last) { df_remote_home(stat_now, state_last); }

	void remote_direction_left(bool stat_now, bool state_last) { df_remote_direction_left(stat_now, state_last); }

	void remote_direction_right(bool stat_now, bool state_last) { df_remote_direction_right(stat_now, state_last); }

	void remote_direction_forward(bool stat_now, bool state_last) { df_remote_direction_forward(stat_now, state_last); }

	void remote_spot(bool stat_now, bool state_last) { df_remote_spot(stat_now, state_last); }

	void remote_max(bool stat_now, bool state_last) { df_remote_max(stat_now, state_last); }

	void rcon(bool stat_now, bool state_last) { df_rcon(stat_now, state_last); }
};

void charge_register_event(void);
void charge_unregister_event(void);
void charge_function();
bool charge_turn_connect(void);
