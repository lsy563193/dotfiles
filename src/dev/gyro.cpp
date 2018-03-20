#include <ros/ros.h>
#include <serial.h>
#include <speaker.h>
#include <beeper.h>
#include <mathematics.h>
#include "gyro.h"
#include "event_manager.h"

Gyro gyro;

Gyro::Gyro(void){
	angle_y_ = 0;
	angle_v_ = 0;
	x_acc_ = -1000;
	y_acc_ = -1000;
	z_acc_ = -1000;
	init_x_acc_ = 0;
	init_y_acc_ = 0;
	init_z_acc_ = 0;
	calibration_status_ = 255;
	status_ = 0;
	tilt_checking_status_ = 0;

	tilt_front_count_ = 0;
	tilt_right_count_ = 0;
	tilt_left_count_ = 0;

	front_count_ = 0;
	left_count_ = 0;
	right_count_ = 0;
	back_count_ = 0;
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
	serial.setSendData(CTL_MIX, static_cast<uint8_t>(serial.getSendData(CTL_MIX) | 0x08));
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
		ROS_DEBUG("Opening%d, angle_v_ = %f.angle = %f.", success_count_, getAngleV(), getAngleY());
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
			auto current_angle_ = getAngleY();
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
	serial.setSendData(CTL_MIX, static_cast<uint8_t>(serial.getSendData(CTL_MIX) & ~0x08));
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
		uint8_t byte = serial.getSendData(CTL_MIX);
		serial.setSendData(CTL_MIX, static_cast<uint8_t>(byte | 0x04));
	}
	//else
	//{
	//	serial.setSendData(CTL_MIX, 0x01);
	//}
}

void Gyro::setDynamicOff(void)
{
	if (isOn())
	{
		uint8_t byte = serial.getSendData(CTL_MIX);
		serial.setSendData(CTL_MIX, static_cast<uint8_t>(byte & ~0x04));
	}
	//else
	//{
	//	serial.setSendData(CTL_MIX, 0x00);
	//}
}
#endif

int16_t Gyro::getFront() {
#if GYRO_FRONT_X_POS
	return x_acc_;
#elif GYRO_FRONT_X_NEG
	return -x_acc_;
#elif GYRO_FRONT_Y_POS
	return -y_acc_;
#elif GYRO_FRONT_Y_NEG
	return y_acc_;
#endif
}

int16_t Gyro::getLeft() {
#if GYRO_FRONT_X_POS
	return y_acc_;
#elif GYRO_FRONT_X_NEG
	return -y_acc_;
#elif GYRO_FRONT_Y_POS
	return x_acc_;
#elif GYRO_FRONT_Y_NEG
	return -x_acc_;
#endif
}

int16_t Gyro::getRight() {
#if GYRO_FRONT_X_POS
	return y_acc_;
#elif GYRO_FRONT_X_NEG
	return -y_acc_;
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
	uint8_t tmp_status = 0;

	if (tilt_checking_enable_)
	{
		//robot front tilt
		if (getFront() - getInitXAcc() > FRONT_TILT_LIMIT)
		{
			tilt_front_count_ +=2;
			ROS_INFO("\033[1;40;32m%s %d: front(%d)\tfront init(%d), front cnt(%d).\033[0m", __FUNCTION__, __LINE__, getFront(), getInitXAcc(), tilt_front_count_);
		}
		else
		{
			if (tilt_front_count_ > 0)
				tilt_front_count_--;
		}
	
		//robot left tilt
		if (getYAcc() - getInitYAcc() > LEFT_TILT_LIMIT)
		{
			tilt_left_count_+=3;
			ROS_INFO("\033[1;40;34m %s %d: left(%d)\tleft init(%d), left cnt(%d).\033[0m", __FUNCTION__, __LINE__, getLeft(), getInitYAcc(), tilt_left_count_);
		}
		else
		{
			if (tilt_left_count_ > 0)
				tilt_left_count_--;
		}

		//robot right tilt
		if (getYAcc() - getInitYAcc() < -RIGHT_TILT_LIMIT)
		{
			tilt_right_count_+=3;
			ROS_INFO("\033[1;40;35m%s %d: right(%d)\tright init(%d), right cnt(%d).\033[0m", __FUNCTION__, __LINE__, getRight(), getInitYAcc(), tilt_right_count_);
		}
		else
		{
			if (tilt_right_count_ > 0)
				tilt_right_count_--;
		}

		if ( tilt_right_count_ >= TILT_COUNT_REACH
				   	|| tilt_left_count_ >= TILT_COUNT_REACH
					|| tilt_front_count_ >= TILT_COUNT_REACH)
		{
			if (tilt_left_count_ >= TILT_COUNT_REACH)
				tmp_status |= TILT_LEFT;
			if (tilt_right_count_ >= TILT_COUNT_REACH)
				tmp_status |= TILT_RIGHT;
			if (tilt_front_count_ >= TILT_COUNT_REACH)
				tmp_status |= TILT_FRONT;
			tilt_front_count_ = 0;
			tilt_left_count_ = 0;
			tilt_right_count_ = 0;
			tilt_back_count_ = 0;
			tilt_z_count_ = 0;
//			beeper.beepForCommand(VALID);
			setTiltCheckingStatus(tmp_status);

			ROS_INFO("\033[47;34m" "%s,%d,robot tilt detect!! tmp_state=%d" "\033[0m",__FUNCTION__,__LINE__,tmp_status);
		}
		else
			setTiltCheckingStatus(0);
	}
	else{
		tilt_front_count_ = 0;
		tilt_left_count_ = 0;
		tilt_right_count_ = 0;
		tilt_back_count_ = 0;
		tilt_z_count_ = 0;
		setTiltCheckingStatus(0);
	}

	return tmp_status;
}

uint8_t Gyro::checkTilt(int front_tilt_limit,int back_tilt_limit,int right_tilt_limit,int left_tilt_limit, int count_reach)
{
	uint8_t tmp_status = 0;

	if (tilt_checking_enable_)
	{
		//robot front tilt
		if (getFront() - getInitXAcc() > front_tilt_limit)
		{
			front_count_ +=2;
		}
		else
		{
			if (front_count_ > 0)
				front_count_--;
		}

		//robot back tilt
		if(getFront() - getInitXAcc() < -back_tilt_limit)
		{
			back_count_ +=2;
		}
		else
		{
			if (back_count_ > 0)
				back_count_--;
		}

		//robot left tilt
		if (getYAcc() - getInitYAcc() > left_tilt_limit)
		{
			left_count_+=3;
		}
		else
		{
			if (left_count_ > 0)
				left_count_--;
		}

		//robot right tilt
		if (getYAcc() - getInitYAcc() < -right_tilt_limit)
		{
			right_count_+=3;
			ROS_INFO("\033[1;40;35m%s %d: right(%d)\tright init(%d), right cnt(%d).\033[0m", __FUNCTION__, __LINE__, getRight(), getInitYAcc(), tilt_right_count_);
		}
		else
		{
			if (right_count_ > 0)
				right_count_--;
		}

		if ( right_count_ >= count_reach || left_count_ >= count_reach || front_count_ >= count_reach+4  || back_count_ >= count_reach +4 )
		{
			front_count_ = 0;
			left_count_ = 0;
			right_count_ = 0;
			back_count_ = 0;
//			beeper.beepForCommand(VALID);
			return true;
		}
	}
	else{
		front_count_ = 0;
		left_count_ = 0;
		right_count_ = 0;
		back_count_ = 0;
		setTiltCheckingStatus(0);
	}

	return false;
}

bool Gyro::isTiltCheckingEnable()
{
	return tilt_checking_enable_;
}

void Gyro::setTiltCheckingEnable(bool val)
{
	tilt_checking_enable_ = val;
}

float Gyro::calAngleR1OrderFilter(double k, double dt)
{
	auto acc_angle = getAccAngleR();
	auto angle = k * acc_angle + (1 - k) * (angle_r_ + angle_v_ * dt);
//	ROS_INFO("angle_r_ = %f, acc_angle= %f, angle_v_= %f", angle_r_,acc_angle,angle_v_);
	return angle;
}
float Gyro::getAngleR(void)
{
//	ROS_INFO("angle_r_(with ofset) = %f", angle_r_ - ANGLE_R_OFFSET_);
	return angle_r_ - ANGLE_R_OFFSET_;
}

float Gyro::calAngleRKalmanFilter(double dt)
{
	auto acc_angle = getAccAngleR();
	auto angle_kalman = KalmanFilter(acc_angle, angle_v_, dt);

//	ROS_INFO("angle_kalman = %f, acc_angle= %f, angle_v_= %f", angle_kalman,acc_angle,angle_v_);
//	printf("%f,%lf.", acc_angle,angle_v_);

	return angle_kalman;
}

//KalmanFilter
float Gyro::KalmanFilter(float angle_m, float gyro_m, double dt)//angleAx和gyroGy
{
	kalman_angle += (gyro_m - q_bias) * dt;
	angle_err = angle_m - kalman_angle;
	Pdot[0] = Q_angle - P[0][1] - P[1][0];
	Pdot[1] = -P[1][1];
	Pdot[2] = -P[1][1];
	Pdot[3] = Q_gyro;
	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;
	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];
	E = R_angle + C_0 * PCt_0;
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];
	P[0][0] -= K_0 * t_0;
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
	kalman_angle += K_0 * angle_err;//best angle
	q_bias += K_1 * angle_err;
	angle_dot = gyro_m - q_bias;//best angular speed
	return kalman_angle;
}

void Gyro::resetKalmanParam(void)
{
	kalman_angle = angle_dot = 0;
//	float temp_P[2][2]={{1,0},{0,1}};
	float temp_P[2][2]={{1,0},{0,1}};
	memcpy(P, temp_P, sizeof(P));
	float temp_Pdot[4]={0,0,0,0};
	memcpy(Pdot, temp_Pdot, sizeof(Pdot));
//	Q_angle=0.001,Q_gyro=0.005;//
//	R_angle=0.5;
	Q_angle=0.03,Q_gyro=0.00009;//0.05
	R_angle=0.0001;//0.5
	C_0=1;
	q_bias=angle_err=PCt_0=PCt_1=E=K_0=K_1=t_0=t_1=0;
}

void Gyro::setAngleR(float angle)
{
	angle_r_ = angle;
}

float Gyro::getAccAngleR(void)
{
	acc_angle_r_ = radian_to_degree(atan2(x_acc_, z_acc_));
//	ROS_WARN("acc(%f, %f, %f)", x_acc_, y_acc_, z_acc_);
	return acc_angle_r_;
}

float Gyro::getGyroAngleR(double dt)
{
	gyro_angle_r_ += angle_v_ * dt;
	return gyro_angle_r_;
}

void Gyro::resetGyroAngleR(void)
{
	gyro_angle_r_ = 0;
}

void Gyro::setAngleVOffset(void)
{
	ANGLE_V_OFFSET_ = angle_v_;
	ROS_ERROR("setAngleVOffset = %f", ANGLE_V_OFFSET_);
}

void Gyro::setAngleROffset(void)
{
	ANGLE_R_OFFSET_ = angle_r_;
	ROS_ERROR("ANGLE_R_OFFSET_ = %f", ANGLE_R_OFFSET_);
}
