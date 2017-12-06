//
// Created by root on 11/20/17.
//
#include "ros/ros.h"
#include "error.h"
#include "speaker.h"
#include "cliff.h"
#include "bumper.h"


Error error;

void Error::alarm(void)
{
	switch (get())
	{
		case ERROR_CODE_LEFTWHEEL:
		{
			speaker.play(SPEAKER_ERROR_LEFT_WHEEL);
			break;
		}
		case ERROR_CODE_RIGHTWHEEL:
		{
			speaker.play(SPEAKER_ERROR_RIGHT_WHEEL);
			break;
		}
		case ERROR_CODE_LEFTBRUSH:
		{
			speaker.play(SPEAKER_ERROR_LEFT_BRUSH);
			break;
		}
		case ERROR_CODE_RIGHTBRUSH:
		{
			speaker.play(SPEAKER_ERROR_RIGHT_BRUSH);
			break;
		}
		case ERROR_CODE_MAINBRUSH:
		{
			speaker.play(SPEAKER_ERROR_MAIN_BRUSH);
			break;
		}
		case ERROR_CODE_FAN_H:
		{
			speaker.play(SPEAKER_ERROR_SUCTION_FAN);
			break;
		}
		case ERROR_CODE_CLIFF:
		{
			speaker.play(SPEAKER_ERROR_CLIFF);
			break;
		}
		case ERROR_CODE_BUMPER:
		{
			speaker.play(SPEAKER_ERROR_BUMPER);
			break;
		}
		case ERROR_CODE_LIDAR:
		{
			speaker.play(SPEAKER_TEST_LIDAR);
			break;
		}
		case ERROR_CODE_STUCK:
		{
			speaker.play(SPEAKER_ROBOT_STUCK);
			break;
		}
		default:
		{
			break;
		}
	}

}

bool Error::clear(uint8_t code)
{
	bool cleared = true;
	switch (code)
	{
		case ERROR_CODE_CLIFF:
		{
			if (cliff.get_status())
			{
				ROS_WARN("%s %d: Cliff still triggered.", __FUNCTION__, __LINE__);
				cleared = false;
			}
			break;
		}
		case ERROR_CODE_BUMPER:
		{
			if (bumper.get_status())
			{
				ROS_WARN("%s %d: Bumper still triggered.", __FUNCTION__, __LINE__);
				cleared = false;
			}
			break;
		}
		default:
			break;
	}

	if (cleared)
		set(ERROR_CODE_NONE);

	return cleared;
}

