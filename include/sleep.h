#include <stdint.h>
#include <unistd.h>

void sleep_mode(void);

void sleep_register_events(void);

void sleep_unregister_events(void);

#define define_sleep_handle_func(name) \
	void sleep_handle_ ## name(bool state_now, bool state_last);

/* Rcon */
define_sleep_handle_func(rcon)
/* Remote */
define_sleep_handle_func(remote_clean)
///* Key */
//define_sleep_handle_func(key_clean)
/* Charge Status */
define_sleep_handle_func(charge_detect)
