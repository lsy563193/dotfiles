//
// Created by root on 11/17/17.
//

#ifndef PP_ACCELERATOR_H
#define PP_ACCELERATOR_H
#include <pp/x900sensor.h>
extern pp::x900sensor sensor;

class Accelerator {
public:
	int16_t get_front() {
#if GYRO_FRONT_X_POS
		return -sensor.x_acc;
#elif GYRO_FRONT_X_NEG
		return sensor.x_acc;
#elif GYRO_FRONT_Y_POS
		return -sensor.y_acc);
#elif GYRO_FRONT_Y_NEG
		return sensor.y_acc;
#endif
	}

	int16_t get_left() {
#if GYRO_FRONT_X_POS
		return -sensor.y_acc;
#elif GYRO_FRONT_X_NEG
		return sensor.y_acc;
#elif GYRO_FRONT_Y_POS
		return sensor.x_acc;
#elif GYRO_FRONT_Y_NEG
		return -sensor.x_acc;
#endif
	}

	int16_t get_right() {
#if GYRO_FRONT_X_POS
		return -sensor.y_acc;
#elif GYRO_FRONT_X_NEG
		return sensor.y_acc;
#elif GYRO_FRONT_Y_POS
		return sensor.x_acc;
#elif GYRO_FRONT_Y_NEG
		return -sensor.x_acc;
#endif
	}

	int16_t get_front_init() {
#if GYRO_FRONT_X_POS
		return -init_xcc_;
#elif GYRO_FRONT_X_NEG
		return init_xcc_;
#elif GYRO_FRONT_Y_POS
		return -init_ycc_;
#elif GYRO_FRONT_Y_NEG
		return init_ycc_;
#endif
	}

	int16_t get_left_init() {
#if GYRO_FRONT_X_POS
		return -init_ycc_;
#elif GYRO_FRONT_X_NEG
		return init_ycc_;
#elif GYRO_FRONT_Y_POS
		return init_xcc_;
#elif GYRO_FRONT_Y_NEG
		return -init_xcc_;
#endif
	}

	int16_t get_right_init() {
#if GYRO_FRONT_X_POS
		return -init_ycc_;
#elif GYRO_FRONT_X_NEG
		return init_ycc_;
#elif GYRO_FRONT_Y_POS
		return init_xcc_;
#elif GYRO_FRONT_Y_NEG
		return -init_xcc_;
#endif
	}

	int16_t getXAcc() const
	{
		return sensor.x_acc;
	}

	int16_t getYAcc() const
	{
		return sensor.y_acc;
	}

	int16_t getZAcc() const
	{
		return sensor.z_acc;
	}

	int16_t getInitXAcc() const
	{
		return init_xcc_;
	}

	int16_t getInitYAcc() const
	{
		return init_ycc_;
	}

	int16_t getInitZAcc() const
	{
		return init_zcc_;
	}

	void setInitXAcc(int16_t val)
	{
		init_xcc_ = val;
	}

	void setInitYAcc(int16_t val)
	{
		init_ycc_ = val;
	}

	void setInitZAcc(int16_t val)
	{
		init_zcc_ = val;
	}
void setAccInitData()
{
	uint8_t count = 0;
	int16_t temp_x_acc = 0;
	int16_t temp_y_acc = 0;
	int16_t temp_z_acc = 0;
	for (count = 0 ; count < 10 ; count++)
	{
		temp_x_acc += getXAcc();
		temp_y_acc += getYAcc();
		temp_z_acc += getZAcc();
		usleep(20000);
	}

	setInitXAcc(temp_x_acc / count);
	setInitYAcc(temp_y_acc / count);
	setInitZAcc(temp_z_acc / count);
//	ROS_INFO("x y z acceleration init val(\033[32m%d,%d,%d\033[0m)" , getInitXAcc(), getInitYAcc(), getInitZAcc());
}
private:
	int16_t init_xcc_;
	int16_t init_ycc_;
	int16_t init_zcc_;

};

extern Accelerator acc;
#endif //PP_ACCELERATOR_H
