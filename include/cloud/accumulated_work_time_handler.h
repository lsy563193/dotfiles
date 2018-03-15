#pragma once

#include "robot.h"
#include "stopwatch.h"

namespace cloud
{

class AccumulatedWorkTimeHandler
{
public:
	AccumulatedWorkTimeHandler();
	~AccumulatedWorkTimeHandler();

	void onStart( void );
	void onStop( void );

private:
	Robot *p_s_robot_;
	Stopwatch stopwatch_;
};

}
