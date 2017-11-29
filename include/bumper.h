//
// Created by root on 11/17/17.
//

#ifndef PP_BUMPER_H
#define PP_BUMPER_H

class Bumper{
public:
	Bumper()
	{
		left_bumper_status_ = false;
		right_bumper_status_ = false;
		lidar_bumper_status_ = false;
		lidar_bumper_fd_ = -1;
		lidar_bumper_activated_ = false;
	}

	uint8_t get_status(void);

	bool getLeft() const
	{
		return left_bumper_status_;
	}

	void setLeft(bool status)
	{
		left_bumper_status_ = status;
	}

	bool getRight() const
	{
		return right_bumper_status_;
	}

	void setRight(bool status)
	{
		right_bumper_status_ = status;
	}

	void setLidarBumperStatus();

	bool getLidarBumperStatus()
	{
		return lidar_bumper_status_;
	}

	int8_t lidarBumperInit(const char *device);
	int8_t lidarBumperDeinit();

private:
	bool left_bumper_status_;
	bool right_bumper_status_;
	bool lidar_bumper_status_;

	// For lidar bumper.
	int lidar_bumper_fd_;
	bool lidar_bumper_activated_;

};

extern Bumper bumper;

#endif //PP_BUMPER_H
