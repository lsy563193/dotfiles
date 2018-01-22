//
// Created by austin on 17-11-24.
//

#ifndef PP_POSE_H
#define PP_POSE_H

class Pose
{
public:
	Pose();
	~Pose();

	void setX(float x);
	float getX(void);
	void setY(float y);
	float getY(void);
	void setZ(float z);
	float getZ(void);
	void setYaw(double yaw);
	double getYaw(void);

private:
	// In meters.
	float x_;
	float y_;
	float z_;

	// In radian.
	double yaw_;
};

#endif //PP_POSE_H
