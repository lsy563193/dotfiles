#include <stdint.h>
#include <unistd.h>
#include <event_manager.h>

void sleep_mode(void);

class Sleep_EventHandle:public EventHandle {

	void rcon(bool state_now, bool state_last);

	void remoteClean(bool state_now, bool state_last);

	void remotePlan(bool state_now, bool state_last);

	void keyClean(bool state_now, bool state_last);

	void chargeDetect(bool state_now, bool state_last);

};
void sleep_register_events(void);

void sleep_unregister_events(void);


