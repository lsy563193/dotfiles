//

#include <fcntl.h>
#include "ros/ros.h"
#include "bumper.h"
#include "map.h"

Bumper bumper;

uint8_t Bumper::get_status(void)
{
	uint8_t temp_status = 0;

	if (getLeft()) {
		temp_status |= BLOCK_LEFT;
	}
	if (getRight()) {
		temp_status |= BLOCK_RIGHT;
	}
	if (getLidarBumperStatus()) {
		temp_status |= BLOCK_FRONT;
	}
	if (temp_status == (BLOCK_LEFT | BLOCK_RIGHT) || (temp_status & BLOCK_FRONT) != 0)
		temp_status = BLOCK_ALL;
	return temp_status;
}

int8_t Bumper::lidarBumperInit(const char *device)
{
	if(lidar_bumper_fd_ != -1)
		return 0;
	char buf[128];
	sprintf(buf,"%s",device);
	lidar_bumper_fd_ = open(buf, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(lidar_bumper_fd_ > 0)
	{
		lidar_bumper_activated_ = true;
		ROS_INFO("%s,%d,open %s success!",__FUNCTION__,__LINE__,buf);
		return 1;
	}
	else if(lidar_bumper_fd_ <= 0 )
	{
		lidar_bumper_fd_ = -1;
		lidar_bumper_activated_ = true;
		ROS_INFO("%s,%d,no such file \033[32m %s \033[0m open fail!!",__FUNCTION__,__LINE__,buf);
		return -1;
	}

}

void Bumper::setLidarBumperStatus()
{
	if(lidar_bumper_activated_ && lidar_bumper_fd_ != -1)
	{
		uint8_t temp_buf[32] = {0};
		if (read(lidar_bumper_fd_,temp_buf,32) >= 0)
		{
			//ROS_INFO("read lidar bumper value:\033[32m%d\033[0m",temp_buf[12]);
			if (temp_buf[12] == 1)
			{
				lidar_bumper_status_ = true;
				return;
			}
			else if (temp_buf[12] == 0)
			{
				lidar_bumper_status_ = false;
				return;
			}
		}
	}
	return;
}

int8_t Bumper::lidarBumperDeinit()
{
	if(lidar_bumper_fd_ == -1)
		return 0;
	int c_ret;
	c_ret = close(lidar_bumper_fd_);
	if(c_ret > 0)
	{
		lidar_bumper_fd_ = -1;
		lidar_bumper_activated_ = 0;
	}
	return c_ret;
}

