//
// Created by austin on 17-11-24.
//

#include "pose.h"

Pose::Pose():x_(0), y_(0), z_(0), angle_(0)
{
};

Pose::~Pose()
{
};

void Pose::setX(float x)
{
	x_ = x;
}

float Pose::getX(void)
{
	return x_;
}

void Pose::setY(float y)
{
	y_ = y;
}

float Pose::getY(void)
{
	return y_;
}

void Pose::setZ(float z)
{
	z_ = z;
}

float Pose::getZ(void)
{
	return z_;
}

void Pose::setAngle(float angle)
{
	angle_ = angle;
}

float Pose::getAngle(void)
{
	return angle_;
}
