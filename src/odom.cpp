//
// Created by austin on 17-11-24.
//

#include "odom.h"
#include "ros/ros.h"

Odom odom;

Odom::Odom():moving_speed_(0), angle_speed_(0), angle_offset_(0)
{
}

Odom::~Odom()
{
}

void Odom::setX(float x)
{
	pose.setX(x);
}

float Odom::getX(void)
{
	return pose.getX();
}

void Odom::setY(float y)
{
	pose.setY(y);
}

float Odom::getY(void)
{
	return pose.getY();
}

void Odom::setZ(float z)
{
	pose.setZ(z);
}

float Odom::getZ(void)
{
	return pose.getZ();
}

void Odom::setAngle(float angle)
{
	pose.setAngle(angle + getAngleOffset());
}

float Odom::getAngle(void)
{
	return pose.getAngle();
}

void Odom::setAngleOffset(float angle)
{
	angle_offset_ = angle;
	ROS_INFO("%s %d: Set odom angle offset to %f.", __FUNCTION__, __LINE__, angle_offset_);
}

float Odom::getAngleOffset(void)
{
	return angle_offset_;
}

void Odom::setMovingSpeed(float speed)
{
//	ROS_INFO("%s %d: Moving speed %f.", __FUNCTION__, __LINE__, moving_speed_);
	moving_speed_ = speed;
}

float Odom::getMovingSpeed(void)
{
	return moving_speed_;
}

void Odom::setAngleSpeed(float speed)
{
	angle_speed_ = speed;
}

float Odom::getAngleSpeed(void)
{
	return angle_speed_;
}
