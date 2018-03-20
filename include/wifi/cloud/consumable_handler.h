#pragma once

#include "robot.h"
#include "stopwatch.h"

namespace cloud
{

class ConsumableHandler
{
public:
	ConsumableHandler();
	~ConsumableHandler();

	void onStart( void );
	void onStop( void );

private:
	Robot *p_s_robot_;
	Stopwatch stopwatch_;
};

}
