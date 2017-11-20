//
// Created by root on 11/17/17.
//

#include "pp.h"
#include "planer.h"

Timer timer;

/*----------------------- Work Timer functions--------------------------*/
static time_t g_start_work_time;
void reset_work_time()
{
	g_start_work_time = time(NULL);
}

uint32_t get_work_time()
{
	return (uint32_t) difftime(time(NULL), g_start_work_time);
}

