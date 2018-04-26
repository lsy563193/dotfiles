//
// Created by root on 11/20/17.
//
#include <cliff.h>
#include <bumper.h>
#include <map.h>
#include "error.h"
#include "speaker.h"
#include "wifi/wifi.h"

Error error;

void Error::alarm(void)
{
	switch (get())
	{
		case ERROR_CODE_LEFTWHEEL:
		{
			speaker.play(VOICE_ERROR_LEFT_WHEEL);
			break;
		}
		case ERROR_CODE_RIGHTWHEEL:
		{
			speaker.play(VOICE_ERROR_RIGHT_WHEEL);
			break;
		}
		case ERROR_CODE_LEFTBRUSH:
		{
			speaker.play(VOICE_ERROR_LEFT_BRUSH);
			break;
		}
		case ERROR_CODE_RIGHTBRUSH:
		{
			speaker.play(VOICE_ERROR_RIGHT_BRUSH);
			break;
		}
		case ERROR_CODE_MAINBRUSH:
		{
			speaker.play(VOICE_ERROR_MAIN_BRUSH);
			break;
		}
		case ERROR_CODE_VACUUM:
		{
			speaker.play(VOICE_ERROR_SUCTION_FAN);
			break;
		}
		case ERROR_CODE_CLIFF:
		{
			speaker.play(VOICE_ERROR_CLIFF);
			break;
		}
		case ERROR_CODE_BUMPER:
		{
			speaker.play(VOICE_ERROR_BUMPER);
			break;
		}
		case ERROR_CODE_LIDAR:
		case ERROR_CODE_SLAM:
		{
			speaker.play(VOICE_TEST_LIDAR);
			break;
		}
		case ERROR_CODE_STUCK:
		{
			speaker.play(VOICE_ROBOT_TRAPPED);
			break;
		}
		case ERROR_CODE_GYRO:
		{
			//todo: speaker.play(VOICE_GYRO_ERROR);
			break;
		}
		default:
		{
			break;
		}
	}
	//-- upload state
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);

}

bool Error::clear(uint8_t code, bool force_clear)
{

	if (force_clear)
	{
		error.set(ERROR_CODE_NONE);
		return true;
	}

	bool cleared = true;
	switch (code)
	{
		case ERROR_CODE_CLIFF:
		{
			if (cliff.getStatus())
			{
				ROS_WARN("%s %d: Cliff still triggered.", __FUNCTION__, __LINE__);
				cleared = false;
			}
			break;
		}
		case ERROR_CODE_BUMPER:
		{
			if (bumper.getStatus() & (BLOCK_LEFT | BLOCK_RIGHT | BLOCK_ALL))
			{
				ROS_WARN("%s %d: Bumper still triggered.", __FUNCTION__, __LINE__);
				cleared = false;
			}
			break;
		}
		case ERROR_CODE_LIDAR:
		{
			if (bumper.getStatus() & (BLOCK_LIDAR_BUMPER))
			{
				ROS_WARN("%s %d: Lidar Bumper still triggered.", __FUNCTION__, __LINE__);
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

