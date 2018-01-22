//
// Created by austin on 17-11-24.
//

#include <mathematics.h>
#include "odom.h"
#include "ros/ros.h"

Odom odom;

Odom::Odom():moving_speed_(0), angle_speed_(0), yaw_offset_(0)
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

void Odom::setYaw(double yaw)
{
	pose.setYaw(yaw + getYawOffset());
}

double Odom::getYaw(void)
{
	return pose.getYaw();
}

void Odom::setYawOffset(double yaw)
{
	yaw_offset_ = yaw;
	ROS_INFO("%s %d: Set odom yaw offset to %f(%f degrees).", __FUNCTION__, __LINE__, yaw_offset_, yaw_offset_*180/PI);
}

double Odom::getYawOffset(void)
{
	return yaw_offset_;
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
