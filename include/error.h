//
// Created by root on 11/20/17.
//

#ifndef PP_ERROR_H
#define PP_ERROR_H

#include "dev.h"
class Error {
public:
	Error() {code =0;}
void set(uint8_t code)
{
	code = code;
}

uint8_t get()
{
	return code;
}

void alarm(void)
{
	switch (get())
	{
		case Error_Code_LeftWheel:
		{
			wav.play(WAV_ERROR_LEFT_WHEEL);
			break;
		}
		case Error_Code_RightWheel:
		{
			wav.play(WAV_ERROR_RIGHT_WHEEL);
			break;
		}
		case Error_Code_LeftBrush:
		{
			wav.play(WAV_ERROR_LEFT_BRUSH);
			break;
		}
		case Error_Code_RightBrush:
		{
			wav.play(WAV_ERROR_RIGHT_BRUSH);
			break;
		}
		case Error_Code_MainBrush:
		{
			wav.play(WAV_ERROR_MAIN_BRUSH);
			break;
		}
		case Error_Code_Fan_H:
		{
			wav.play(WAV_ERROR_SUCTION_FAN);
			break;
		}
		case Error_Code_Cliff:
		{
			wav.play(WAV_ERROR_CLIFF);
			break;
		}
		case Error_Code_Bumper:
		{
			wav.play(WAV_ERROR_BUMPER);
			break;
		}
		case Error_Code_Omni:
		{
			wav.play(WAV_ERROR_MOBILITY_WHEEL);
			break;
		}
		case Error_Code_Laser:
		{
			wav.play(WAV_TEST_LIDAR);
			break;
		}
		case Error_Code_Stuck:
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

bool clear(uint8_t code)
{
	bool cleared = true;
	switch (code)
	{
		case Error_Code_LeftWheel:
		case Error_Code_RightWheel:
		case Error_Code_LeftBrush:
		case Error_Code_RightBrush:
		case Error_Code_MainBrush:
		case Error_Code_Fan_H:
			break;
		case Error_Code_Cliff:
		{
			if (cliff.get_status())
			{
				ROS_WARN("%s %d: Cliff still triggered.", __FUNCTION__, __LINE__);
				cleared = false;
			}
			break;
		}
		case Error_Code_Bumper:
		{
			if (bumper.get_status())
			{
				ROS_WARN("%s %d: Bumper still triggered.", __FUNCTION__, __LINE__);
				cleared = false;
			}
			break;
		}
		case Error_Code_Omni:
		{
			if(omni.stop())
			{
				ROS_WARN("%s %d: Omni still triggered.", __FUNCTION__, __LINE__);
				cleared = false;
			}
			break;
		}
		default:
			break;
	}

	return cleared;
}

private:
	uint8_t code;
};

extern Error error;
#endif //PP_ERROR_H
