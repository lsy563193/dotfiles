//
// Created by root on 11/20/17.
//
#include "ros/ros.h"
#include "error.h"
#include "wav.h"
#include "cliff.h"
#include "bumper.h"


Error error;

void Error::alarm(void)
{
	switch (get())
	{
		case ERROR_CODE_LEFTWHEEL:
		{
			wav.play(WAV_ERROR_LEFT_WHEEL);
			break;
		}
		case ERROR_CODE_RIGHTWHEEL:
		{
			wav.play(WAV_ERROR_RIGHT_WHEEL);
			break;
		}
		case ERROR_CODE_LEFTBRUSH:
		{
			wav.play(WAV_ERROR_LEFT_BRUSH);
			break;
		}
		case ERROR_CODE_RIGHTBRUSH:
		{
			wav.play(WAV_ERROR_RIGHT_BRUSH);
			break;
		}
		case ERROR_CODE_MAINBRUSH:
		{
			wav.play(WAV_ERROR_MAIN_BRUSH);
			break;
		}
		case ERROR_CODE_FAN_H:
		{
			wav.play(WAV_ERROR_SUCTION_FAN);
			break;
		}
		case ERROR_CODE_CLIFF:
		{
			wav.play(WAV_ERROR_CLIFF);
			break;
		}
		case ERROR_CODE_BUMPER:
		{
			wav.play(WAV_ERROR_BUMPER);
			break;
		}
		case ERROR_CODE_LASER:
		{
			wav.play(WAV_TEST_LIDAR);
			break;
		}
		case ERROR_CODE_STUCK:
		{
			wav.play(WAV_ROBOT_STUCK);
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
		case ERROR_CODE_LEFTWHEEL:
		case ERROR_CODE_RIGHTWHEEL:
		case ERROR_CODE_LEFTBRUSH:
		case ERROR_CODE_RIGHTBRUSH:
		case ERROR_CODE_MAINBRUSH:
		case ERROR_CODE_FAN_H:
			break;
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

	return cleared;
}

