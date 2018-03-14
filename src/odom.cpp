//
// Created by austin on 17-11-24.
//

#include <mathematics.h>
#include "odom.h"
#include "ros/ros.h"

Odom odom;

Odom::Odom():moving_speed_(0), angle_speed_(0), radian_offset_(0)
{
}

Odom::~Odom()
{
}

void Odom::setX(float x)
{
	pose.setX(x + x_offset_);
}

float Odom::getX(void)
{
	return pose.getX();
}

void Odom::setY(float y)
{
	pose.setY(y+y_offset_);
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

void Odom::setRadian(double radian)
{
	pose.setRadian(radian + getRadianOffset());
}

double Odom::getRadian(void)
{
	return pose.getRadian();
}

void Odom::setRadianOffset(double radian)
{
	radian_offset_ = radian;
	ROS_INFO("%s %d: Set x offset to (%f degrees).", __FUNCTION__, __LINE__, radian_to_degree(radian_offset_));
}
void Odom::setXOffset(double x)
{
	x_offset_ = x;
	ROS_INFO("%s %d: Set y offset to (%f).", __FUNCTION__, __LINE__, x_offset_);
}
void Odom::setYOffset(double y)
{
	y_offset_ = y;
	ROS_INFO("%s %d: Set y offset to (%f).", __FUNCTION__, __LINE__, y_offset_);
}

double Odom::getRadianOffset(void)
{
	return radian_offset_;
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
