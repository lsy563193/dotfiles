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
	}

	bool waitForOn(void);

	bool isStable(void);

	void setOff(void);

	void setOn(void);

	void reOpen(void);

	void setStatus(void);

	void resetStatus(void);

	bool isOn(void);

#if GYRO_DYNAMIC_ADJUSTMENT

	void setDynamicOn(void);

	void setDynamicOff(void);

#endif

	int16_t getXAcc(void);

	void setXAcc(int16_t x_acc);

	int16_t getYAcc(void);

	void setYAcc(int16_t y_acc);

	int16_t getZAcc(void);

	void setZAcc(int16_t z_acc);

	uint8_t getCalibration(void);

	float getAngle(void);

	void setAngle(float angle);

	float getAngleV();

	void setAngleV(float angle_v);

	int16_t getInitXAcc() const;

	int16_t getInitYAcc() const;

	int16_t getInitZAcc() const;

	void setInitXAcc(int16_t val);

	void setInitYAcc(int16_t val);

	void setInitZAcc(int16_t val);

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
};

extern Gyro gyro;
#endif /* __GYRO_H */
