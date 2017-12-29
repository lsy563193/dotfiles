//
// Created by root on 11/17/17.
//

#ifndef PP_LED_H
#define PP_LED_H

// For LED control
// LED type
#define LED_STEADY					0
#define LED_FLASH					1
#define LED_BREATH					2
// LED color
#define LED_OFF						0
#define LED_GREEN					1
#define LED_ORANGE					2
#define LED_RED						3


class Led {
public:
void set(uint16_t G, uint16_t R);
// time_ms is used for both LED_FLASH type and LED_BREATH type, the default value is for LED_BREATH.
void set_mode(uint8_t type, uint8_t color, uint16_t time_ms = 3000);

};

extern Led led;
#endif //PP_LED_H

