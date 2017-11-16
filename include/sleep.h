#include <stdint.h>
#include <unistd.h>
#include <event_manager.h>

void sleep_mode(void);

class Sleep_EventHandle:public EventHandle {

	void rcon(bool state_now, bool state_last);

	void remote_clean(bool state_now, bool state_last);

	void remote_plan(bool state_now, bool state_last);

	void key_clean(bool state_now, bool state_last);

	void charge_detect(bool state_now, bool state_last);

};
void sleep_register_events(void);

void sleep_unregister_events(void);

#define define_func(name) \
	void  ## name(bool state_now, bool state_last);

