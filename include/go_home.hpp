#include <unistd.h>
#include <stdint.h>

#define Round_Left			0x01
#define Round_Right			0x02

void go_home(void);
void around_chargerstation(uint8_t Dir);
uint8_t Check_Position(uint8_t Dir);
void By_Path(void);
uint8_t Home_Check_Current(void);
void Home_Motor_Set(void);

