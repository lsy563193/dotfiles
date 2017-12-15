#include <unistd.h>
#include <stdint.h>
#include <event_manager.h>

class Charge_EventHandle:public EventHandle {
	void chargeDetect(bool state_now, bool state_last);

	void remotePlan(bool state_now, bool state_last);

	void keyClean(bool state_now, bool state_last);

	void cliffAll(bool state_now, bool state_last);

	void remoteClean(bool stat_now, bool state_last);

	void remoteHome(bool stat_now, bool state_last) { df_remote_home(stat_now, state_last); }

	void remoteDirectionLeft(bool stat_now, bool state_last) { df_remote_direction_left(stat_now, state_last); }

	void remoteDirectionRight(bool stat_now, bool state_last) { df_remote_direction_right(stat_now, state_last); }

	void remoteDirectionForward(bool stat_now, bool state_last) { df_remote_direction_forward(stat_now, state_last); }

	void remoteSpot(bool stat_now, bool state_last) { df_remote_spot(stat_now, state_last); }

	void remoteMax(bool stat_now, bool state_last) { df_remote_max(stat_now, state_last); }

	void rcon(bool stat_now, bool state_last) { df_rcon(stat_now, state_last); }
};

void charge_function();
bool charge_turn_connect(void);
