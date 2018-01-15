#include <ros/ros.h>
#include <serial.h>
#include <speaker.h>
#include "gyro.h"
#include "event_manager.h"

Gyro gyro;

Gyro::Gyro(void) {
	angle_ = 0;
	angle_v_ = 0;
	x_acc_ = 0;
	y_acc_ = 0;
	z_acc_ = 0;
	init_x_acc_ = 0;
	init_y_acc_ = 0;
	init_z_acc_ = 0;
	calibration_status_ = 255;
	status_ = 0;
	tilt_checking_status_ = 0;
}

void Gyro::setStatus(void)
{
	status_ = true;
}

void Gyro::resetStatus(void)
{
	status_ = false;
}

bool Gyro::isOn(void)
{
	return status_;
}

void Gyro::setOn(void)
{
	serial.setSendData(CTL_GYRO, 0x02);
	if (isOn()){
		ROS_INFO("gyro on already");
	}
	else
	{
		//ROS_INFO("Set gyro on");
		ROS_DEBUG("Set gyro on");
	}
}

void Gyro::reOpen(void)
{
	resetStatus();
	error_count_ = 0;
	success_count_ = 0;
	skip_count_ = 0;
	average_angle_ = 0;
	check_stable_count_ = 0;
	setOff();
	open_state_ = WAIT_FOR_CLOSE;
}

bool Gyro::waitForOn(void)
{
	if (error_count_ > 10)
	{
		ev.fatal_quit = true;
		return false;
	}

	if (open_state_ == WAIT_FOR_CLOSE)
	{
		auto current_angle_v_ = getAngleV();
		if (current_angle_v_ != last_angle_v_)
		{
			setOff();
			check_stable_count_ = 0;
			last_angle_v_ = current_angle_v_;
		}
		else
			check_stable_count_++;

		if (check_stable_count_ > 10)
		{
			check_stable_count_ = 0;
			skip_count_ = 25;
			resetStatus();
			open_state_ = WAIT_FOR_REOPEN;
		}
	}
	else if (open_state_ == WAIT_FOR_REOPEN)
	{
		if (skip_count_ != 0)
			skip_count_--;
		else
		{
			ROS_DEBUG("re-open gyro");
			setOn();
			open_state_ = WAIT_FOR_OPEN;
		}
	}
	else if (open_state_ == WAIT_FOR_OPEN)
	{
		if (getAngleV() != 0)
			success_count_++;
		ROS_DEBUG("Opening%d, angle_v_ = %f.angle = %f.", success_count_, getAngleV(), getAngle());
		if (success_count_ == 5)
		{
			success_count_ = 0;
			open_state_ = WAIT_FOR_STABLE;
		}
	}
	else if (open_state_ == WAIT_FOR_STABLE)
	{
		if (check_stable_count_ < 50)
		{
			auto current_angle_ = getAngle();
			ROS_DEBUG("Checking%d, angle_v_ = %f.angle = %f, average_angle = %f.",
					  check_stable_count_, getAngleV(), current_angle_, average_angle_);
			if (current_angle_ > 0.02 || current_angle_ < -0.02)
			{
				ROS_WARN("%s %d: Robot is moved when opening gyro, re-open gyro, current_angle_ = %f.", __FUNCTION__, __LINE__, current_angle_);
				setOff();
				average_angle_ = 0;
				check_stable_count_ = 0;
				last_angle_v_ = getAngleV();
				error_count_++;
				open_state_ = WAIT_FOR_CLOSE;
				speaker.play(VOICE_SYSTEM_INITIALIZING, false);
			}
			check_stable_count_++;
			average_angle_ = (average_angle_ + current_angle_) / 2;
		}
		else
		{
			if (average_angle_ > 0.002)
			{
				ROS_WARN("%s %d: Robot is moved when opening gyro, re-open gyro, average_angle = %f.", __FUNCTION__, __LINE__, average_angle_);
				setOff();
				average_angle_ = 0;
				check_stable_count_ = 0;
				last_angle_v_ = getAngleV();
				error_count_++;
				open_state_ = WAIT_FOR_CLOSE;
				speaker.play(VOICE_SYSTEM_INITIALIZING, false);
			}
			else
				// Open gyro succeeded.
				setStatus();
		}
	}

}

void Gyro::setOff()
{
	serial.setSendData(CTL_GYRO, 0x00);
	if (!Gyro::isOn()){
		ROS_INFO("gyro stop already");
		return;
	}
}

#if GYRO_DYNAMIC_ADJUSTMENT
void Gyro::setDynamicOn(void)
{
	if (Gyro::isOn())
	{
		uint8_t byte = serial.getSendData(CTL_GYRO);
		serial.setSendData(CTL_GYRO, byte | 0x01);
	}
	//else
	//{
	//	serial.setSendData(CTL_GYRO, 0x01);
	//}
}

void Gyro::setDynamicOff(void)
{
	if (isOn())
	{
		uint8_t byte = serial.getSendData(CTL_GYRO);
		serial.setSendData(CTL_GYRO, byte | 0x00);
	}
	//else
	//{
	//	serial.setSendData(CTL_GYRO, 0x00);
	//}
}
#endif

int16_t Gyro::getFront() {
#if GYRO_FRONT_X_POS
	return -x_acc_;
#elif GYRO_FRONT_X_NEG
	return x_acc_;
#elif GYRO_FRONT_Y_POS
	return -y_acc_;
#elif GYRO_FRONT_Y_NEG
	return y_acc_;
#endif
}

int16_t Gyro::getLeft() {
#if GYRO_FRONT_X_POS
	return -y_acc_;
#elif GYRO_FRONT_X_NEG
	return y_acc_;
#elif GYRO_FRONT_Y_POS
	return x_acc_;
#elif GYRO_FRONT_Y_NEG
	return -x_acc_;
#endif
}

int16_t Gyro::getRight() {
#if GYRO_FRONT_X_POS
	return -y_acc_;
#elif GYRO_FRONT_X_NEG
	return y_acc_;
#elif GYRO_FRONT_Y_POS
	return x_acc_;
#elif GYRO_FRONT_Y_NEG
	return -x_acc_;
#endif
}

int16_t Gyro::getFrontInit() {
#if GYRO_FRONT_X_POS
	return -init_x_acc_;
#elif GYRO_FRONT_X_NEG
	return init_x_acc_;
#elif GYRO_FRONT_Y_POS
	return -init_y_acc_;
#elif GYRO_FRONT_Y_NEG
	return init_y_acc_;
#endif
}

int16_t Gyro::getLeftInit() {
#if GYRO_FRONT_X_POS
	return -init_y_acc_;
#elif GYRO_FRONT_X_NEG
	return init_y_acc_;
#elif GYRO_FRONT_Y_POS
	return init_x_acc_;
#elif GYRO_FRONT_Y_NEG
	return -init_x_acc_;
#endif
}

int16_t Gyro::getRightInit() {
#if GYRO_FRONT_X_POS
	return -init_y_acc_;
#elif GYRO_FRONT_X_NEG
	return init_y_acc_;
#elif GYRO_FRONT_Y_POS
	return init_x_acc_;
#elif GYRO_FRONT_Y_NEG
	return -init_x_acc_;
#endif
}

void Gyro::setAccInitData()
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

uint8_t Gyro::checkTilt()
{
	//todo Change the method of getting the acc data, now data is from gyro instance.
	uint8_t tmp_status = 0;

	if (tilt_checking_enable_)
	{
		if (getFront() - getFrontInit() > FRONT_TILT_LIMIT)
		{
			tilt_front_count_ += 2;
			//ROS_WARN("%s %d: front(%d)\tfront init(%d), front cnt(%d).", __FUNCTION__, __LINE__, getFront(), getFrontInit(), tilt_front_count_);
		}
		else
		{
			if (tilt_front_count_ > 0)
				tilt_front_count_--;
			else
				tilt_front_count_ = 0;
		}
		if (getLeft() - getLeftInit() > LEFT_TILT_LIMIT)
		{
			tilt_left_count_++;
			//ROS_WARN("%s %d: left(%d)\tleft init(%d), left cnt(%d).", __FUNCTION__, __LINE__, getLeft(), getLeftInit(), tilt_left_count_);
		}
		else
		{
			if (tilt_left_count_ > 0)
				tilt_left_count_--;
		}
		if (getRight() - getRightInit() > RIGHT_TILT_LIMIT)
		{
			tilt_right_count_++;
			//ROS_WARN("%s %d: right(%d)\tright init(%d), right cnt(%d).", __FUNCTION__, __LINE__, getRight(), getRightInit(), tilt_right_count_);
		}
		else
		{
			if (tilt_right_count_ > 0)
				tilt_right_count_--;
		}
		if (abs(getZAcc() - getInitZAcc()) > DIF_TILT_Z_VAL)
		{
			tilt_z_count_++;
			//ROS_WARN("%s %d: z(%d)\tzi(%d).", __FUNCTION__, __LINE__, getZAcc(), getInitZAcc());
		}
		else
		{
			if (tilt_z_count_ > 1)
				tilt_z_count_ -= 2;
			else
				tilt_z_count_ = 0;
		}

		//if (left_count > 7 || front_count > 7 || right_count > 7 || z_count > 7)
			//ROS_WARN("%s %d: count left:%d, front:%d, right:%d, z:%d", __FUNCTION__, __LINE__, left_count, front_count, right_count, z_count);

		if (tilt_front_count_ + tilt_left_count_ + tilt_right_count_ + tilt_z_count_ > TILT_COUNT_REACH)
		{
			ROS_INFO("\033[47;34m" "%s,%d,robot tilt !!" "\033[0m",__FUNCTION__,__LINE__);
			if (tilt_left_count_ > TILT_COUNT_REACH / 3)
				tmp_status |= TILT_LEFT;
			if (tilt_right_count_ > TILT_COUNT_REACH / 3)
				tmp_status |= TILT_RIGHT;

			if (tilt_front_count_ > TILT_COUNT_REACH / 3 || !tmp_status)
				tmp_status |= TILT_FRONT;
			setTiltCheckingStatus(tmp_status);
			tilt_front_count_ /= 3;
			tilt_left_count_ /= 3;
			tilt_right_count_ /= 3;
			tilt_z_count_ /= 3;
		}
		else if (tilt_front_count_ + tilt_left_count_ + tilt_right_count_ + tilt_z_count_ < TILT_COUNT_REACH / 4)
			setTiltCheckingStatus(0);
	}
	else{
		tilt_front_count_ = 0;
		tilt_left_count_ = 0;
		tilt_right_count_ = 0;
		tilt_z_count_ = 0;
		setTiltCheckingStatus(0);
	}

	return tmp_status;
}

bool Gyro::isTiltCheckingEnable()
{
	return tilt_checking_enable_;
}

void Gyro::TiltCheckingEnable(bool val)
{
	tilt_checking_enable_ = val;
}

