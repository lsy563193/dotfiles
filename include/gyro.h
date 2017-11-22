#ifndef __GYRO_H__
#define __GYRO_H__

class Gyro {
public:

	Gyro() {
		angle_ = 0;   //[3:3]   = Angle
		xacc = 0;        //[7:8]   = X Acceleration
		yacc = 0;        //[9:10]  = Y Acceleration
		zacc = 0;        //[11:12] = Z Acceleration
		calibration = 255;
		status_ = 0;
	}

	bool wait_for_on(void);

	bool is_stable(void);

	void set_off(void);

	void set_on(void);

	void set_status(void);

	void reset_status(void);

	bool is_on(void);

#if GYRO_DYNAMIC_ADJUSTMENT

	void set_dynamic_on(void);

	void set_dynamic_off(void);

#endif

	int16_t get_x_acc(void);

	int16_t get_y_acc(void);

	int16_t get_z_acc(void);

	uint8_t get_calibration(void);

	int16_t get_angle(void);

	void set_angle(int16_t angle);

private:

	int16_t angle_;   //[3:3]   = Angle
	int16_t xacc;        //[7:8]   = X Acceleration
	int16_t yacc;        //[9:10]  = Y Acceleration
	int16_t zacc;        //[11:12] = Z Acceleration

	uint8_t calibration;

	bool status_;
};

extern Gyro gyro;
#endif /* __GYRO_H */
