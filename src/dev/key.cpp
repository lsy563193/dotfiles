//
// Created by root on 11/17/17.
//

#include "pp.h"

Key key;

void Key::eliminate_jitter(uint8_t k_n)
{
		if ((k_n & KEY_CLEAN) && !(key.get_press() & KEY_CLEAN)) {
			press_count++;
			if (press_count > 0) {
				key.set_press(KEY_CLEAN);
				press_count = 0;
				// When key 'clean' is triggered, it will set touch status.
				key.set();
			}
		}
		else if (!(k_n & KEY_CLEAN) && (key.get_press() & KEY_CLEAN)) {
			release_count++;
			if (release_count > 5) {
				key.reset_press(KEY_CLEAN);
				release_count = 0;
			}
		}
		else {
			press_count = 0;
			release_count = 0;
		}
}
