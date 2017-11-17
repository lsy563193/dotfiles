//

#include "pp.h"
#include "bumper.h"

Bumper bumper;


int8_t bumper_lidar_init(const char *device)
{
	if(lidar_bumper_fd != -1)
		return 0;
	char buf[128];
	sprintf(buf,"%s",device);
	lidar_bumper_fd = open(buf, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(lidar_bumper_fd > 0)
	{
		is_lidar_bumper_init = 1;
		ROS_INFO("%s,%d,open %s success!",__FUNCTION__,__LINE__,buf);
		return 1;
	}
	else if(lidar_bumper_fd <= 0 )
	{
		lidar_bumper_fd = -1;
		is_lidar_bumper_init = 0;
		ROS_INFO("%s,%d,no such file \033[32m %s \033[0m open fail!!",__FUNCTION__,__LINE__,buf);
		return -1;
	}

}

int8_t bumper_get_lidar_status()
{
	if(is_lidar_bumper_init && lidar_bumper_fd != -1)
	{
		uint8_t temp_buf[32] = {0};
		int reval;
		reval = read(lidar_bumper_fd,temp_buf,32);
		if(reval >=0){
			ROS_INFO("read lidar bumper value:\033[32m%d\033[0m",temp_buf[12]);
			return temp_buf[12];
		}
		else
			return -1;
	}
	return 0;
}

int8_t bumper_lidar_deinit()
{
	if(lidar_bumper_fd == -1)
		return 0;
	int c_ret;
	c_ret = close(lidar_bumper_fd);
	if(c_ret > 0)
	{
		lidar_bumper_fd = -1;
		is_lidar_bumper_init = 0;
	}
	return c_ret;
}

