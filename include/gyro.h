#ifndef __GYRO_H__
#define __GYRO_H__

//for tilt detct
#include <cstdint>

#define DIF_TILT_X_VAL				100
#define DIF_TILT_Y_VAL				100
#define DIF_TILT_Z_VAL				100

#define TILT_COUNT_REACH			10
#define FRONT_TILT_LIMIT			180
#define LEFT_TILT_LIMIT				150
#define RIGHT_TILT_LIMIT			150
#define BACK_TILT_LIMIT				280

#define TILT_RIGHT					0x1
#define TILT_FRONT					0x2
#define TILT_LEFT						0x4
#define TILT_BACK						0x8

typedef enum{
	WAIT_FOR_OPEN = 0,
	WAIT_FOR_STABLE,
	WAIT_FOR_REOPEN,
	WAIT_FOR_CLOSE
}GyroState;

class Gyro {
public:
	Gyro(void);

	void setStatus(void);

	void resetStatus(void);

	bool isOn(void);

	void setOn(void);

	void reOpen(void);

	bool waitForOn(void);

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

	uint8_t checkTilt(int front_tilt_limit ,int back_tilt_limit,int right_tilt_limit ,
										int left_tilt_limit , int tilt_count_reach);
	uint8_t checkTilt();

	bool isTiltCheckingEnable(void);

	void setTiltCheckingEnable(bool val);

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

	bool getCalibration(void)
	{
		return calibration_status_;
	}

	void setCalibration(bool val)
	{
		calibration_status_ = val;
	}

	float getAngleY(void)
	{
		return angle_y_;
	}

	void setAngleY(float angle)
	{
		angle_y_ = angle;
	}

	float getAngleV()
	{
		return angle_v_;
	}

	void setAngleV(float angle_v)
	{
		angle_v_ = angle_v - ANGLE_V_OFFSET_;
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

	float calAngleR1OrderFilter(double k, double dt);
	float calAngleRKalmanFilter(double dt);
	void setAngleR(float angle);
	float getAngleR(void);
	float getAccAngleR(void);
	float getGyroAngleR(double dt);
	void resetGyroAngleR(void);
	float KalmanFilter(float angle_m, float gyro_m, double dt);
	void setAngleVOffset(void);
	void setAngleROffset(void);
	void resetKalmanParam(void);
private:

	float angle_y_;//yaw
	float angle_r_{0};//roll
	float angle_p_;//pitch
	float angle_v_;
	float x_acc_;
	float y_acc_;
	float z_acc_;
	float init_x_acc_;
	float init_y_acc_;
	float init_z_acc_;

	float acc_angle_r_{0};
	float gyro_angle_r_{0};
	float ANGLE_V_OFFSET_{0};
	float ANGLE_R_OFFSET_{0};

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
	uint16_t tilt_front_count_;
	uint16_t tilt_back_count_;
	uint16_t tilt_left_count_;
	uint16_t tilt_right_count_;
	uint16_t tilt_z_count_;

	uint16_t front_count_;
	uint16_t right_count_;
	uint16_t left_count_;
	uint16_t back_count_;

	//for kalman
	float kalman_angle, angle_dot;
	float P[2][2]={{1,0},{0,1}};
	float Pdot[4]={0,0,0,0};
	float Q_angle=0.05,Q_gyro=0.005;
	float R_angle=0.5,C_0=1;
	float q_bias,angle_err,PCt_0,PCt_1,E,K_0,K_1,t_0,t_1;
};

extern Gyro gyro;
#endif /* __GYRO_H */
