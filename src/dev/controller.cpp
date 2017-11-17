//
// Created by root on 11/17/17.
//

#include "pp.h"
#include "controller.h"

Controller controller;

static uint8_t g_sendflag = 0;

void set_send_flag(void)
{
	g_sendflag = 1;
}

void reset_send_flag(void)
{
	g_sendflag = 0;
}

