#ifndef __GYRO_H__
#define __GYRO_H__

typedef enum{
	WAIT_FOR_OPEN = 0,
	WAIT_FOR_STABLE,
	WAIT_FOR_REOPEN,
	WAIT_FOR_CLOSE
}GyroState;

class Gyro {
public:

	Gyro() {
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

	void setStatus(void);

	void resetStatus(void);

	bool isOn(void);

	void setOn(void);

	void reOpen(void);

	bool waitForOn(void);

	bool isStable(void);

	void setOff(void);

	int16_t getFront(void);

	int16_t getLeft(void);

	int16_t getRight(void);

	int16_t getFrontInit(void);

	int16_t getLeftInit(void);

	int16_t getRightInit(void);

	void setAccInitData(void);

#if GYRO_DYNAMIC_ADJUSTMENT

	void setDynamicOn(void);

	void setDynamicOff(void);
#endif

	uint8_t checkTilt(void);

	bool isTiltCheckingEnable(void);

	void TiltCheckingEnable(bool val);

	int16_t getXAcc(void)
	{
		return x_acc_;
	}

	void setXAcc(int16_t x_acc)
	{
		x_acc_ = x_acc;
	}

	int16_t getYAcc(void)
	{
		return y_acc_;
	}

	void setYAcc(int16_t y_acc)
	{
		y_acc_ = y_acc;
	}

	int16_t getZAcc(void)
	{
		return z_acc_;
	}

	void setZAcc(int16_t z_acc)
	{
		z_acc_ = z_acc;
	}

	uint8_t getCalibration(void)
	{
		return calibration_status_;
	}

	void setCalibration(uint8_t val)
	{
		calibration_status_ = val;
	}

	float getAngle(void)
	{
		return angle_; 
	}
	void setAngle(float angle)
	{
		angle_ = angle;
	}

	float getAngleV()
	{
		return angle_v_;
	}

	void setAngleV(float angle_v)
	{
		angle_v_ = angle_v;
	}

	int16_t getInitXAcc() const
	{
		return init_x_acc_;
	}

	void setInitXAcc(uint16_t val)
	{
		init_x_acc_ = val;
	}

	int16_t getInitYAcc() const
	{
		return init_y_acc_;
	}

	void setInitYAcc(uint16_t val)
	{
		init_y_acc_ = val;
	}

	int16_t getInitZAcc() const
	{
		return init_z_acc_;
	}

	void setInitZAcc(int16_t val)
	{
		init_z_acc_ = val;
	}

	void setTiltCheckingStatus(uint8_t status)
	{
		tilt_checking_status_ = status;
	}

	uint8_t getTiltCheckingStatus()
	{
		return tilt_checking_status_;
	}

private:

	float angle_;
	float angle_v_;
	float x_acc_;
	float y_acc_;
	float z_acc_;
	float init_x_acc_;
	float init_y_acc_;
	float init_z_acc_;

	bool calibration_status_;

	bool status_;
	GyroState open_state_;
	// For gyro opening
	uint8_t error_count_;
	uint8_t success_count_;
	uint8_t skip_count_;
	float average_angle_;
	uint8_t check_stable_count_;
	float last_angle_v_;
	//for tilt checking
	bool tilt_checking_enable_;
	uint8_t tilt_checking_status_;
};

extern Gyro gyro;
#endif /* __GYRO_H */
