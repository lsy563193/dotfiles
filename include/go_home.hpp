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

