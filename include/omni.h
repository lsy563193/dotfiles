//
// Created by root on 11/20/17.
//

#ifndef PP_OMNI_H
#define PP_OMNI_H

#include "controller.h"
class Omni {
public:
void reset()
{
	uint8_t reset_byte = controller.get(CTL_OMNI_RESET);
	controller.set(CTL_OMNI_RESET, reset_byte | 0x01);
}

void clear()
{
	uint8_t reset_byte = controller.get(CTL_OMNI_RESET);
	controller.set(CTL_OMNI_RESET, reset_byte & ~0x01);
}

private:
};

extern Omni omni;

#endif //PP_OMNI_H
