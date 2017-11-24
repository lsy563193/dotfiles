#ifndef __GYRO_H__
#define __GYRO_H__

class Gyro {
public:

	Gyro() {
		angle_ = 0;   //[3:3]   = Angle
		angle_v_ = 0;
		xacc = 0;        //[7:8]   = X Acceleration
		yacc = 0;        //[9:10]  = Y Acceleration
		zacc = 0;        //[11:12] = Z Acceleration
		calibration = 255;
		status_ = 0;
	}

	bool waitForOn(void);

	bool isStable(void);

	void setOff(void);

	void setOn(void);

	void setStatus(void);

	void resetStatus(void);

	bool isOn(void);

#if GYRO_DYNAMIC_ADJUSTMENT

	void setDynamicOn(void);

	void setDynamicOff(void);

#endif

	int16_t getXAcc(void);

	int16_t getYAcc(void);

	int16_t getZAcc(void);

	uint8_t getCalibration(void);

	float getAngle(void);

	void setAngle(float angle);

	float getAngleV();

	void setAngleV(float angle_v);

private:

	float angle_;   //[3:3]   = Angle
	float angle_v_;   //[3:3]   = Angle
	float x_acc_;        //[7:8]   = X Acceleration
	float y_acc_;        //[9:10]  = Y Acceleration
	float z_acc_;        //[11:12] = Z Acceleration

	bool calibration_status_;

	bool status_;
};

extern Gyro gyro;
#endif /* __GYRO_H */
