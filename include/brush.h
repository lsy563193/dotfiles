//
// Created by root on 11/17/17.
//

#ifndef PP_BRUSH_H
#define PP_BRUSH_H

class Brush {
public:
	Brush(void);

	void slowOperate(void);
	void normalOperate(void);
	void fullOperate(void);
	void stop(void);
	void mainBrushResume();

	bool isOn()
	{
		return brush_status_ != brush_stop;
	}

	void setPWM(uint8_t L, uint8_t R, uint8_t M);

	uint8_t leftIsStall(void);

	uint8_t rightIsStall(void);

	void setLeftOc(bool val)
	{
		is_left_oc_ = val;
	}

	bool getLeftOc() const
	{
		return is_left_oc_;
	}

	void setRightOc(bool val)
	{
		is_right_oc_ = val;
	}

	bool getRightOc() const
	{
		return is_right_oc_;
	}

	void setMainOc(bool val)
	{
		is_main_oc_ = val;
	}

	bool getMainOc() const
	{
		return is_main_oc_;
	}

	void setLeftCurrent(uint16_t current)
	{
		left_current_ = current;
	}

	uint16_t getLeftCurrent()
	{
		return left_current_;
	}

	void setRightCurrent(uint16_t current)
	{
		right_current_ = current;
	}

	uint16_t getRightCurrent()
	{
		return right_current_;
	}

	void setMainCurrent(uint16_t current)
	{
		main_current_ = current;
	}

	uint16_t getMainCurrent()
	{
		return main_current_;
	}

	void checkBatterySetPWM();
	void updatePWM();

	uint8_t oc_left_cnt_;
	uint8_t oc_main_cnt_;
	uint8_t oc_right_cnt_;

private:

	//Value for saving SideBrush_PWM
	enum {
		brush_stop,
		brush_slow,
		brush_normal,
		brush_max,
	};
	double check_battery_time_stamp_;
	uint8_t brush_status_;
	uint8_t normal_PWM;

	bool is_left_oc_;
	bool is_right_oc_;
	bool is_main_oc_;

	uint16_t left_current_;
	uint16_t right_current_;
	uint16_t main_current_;
};
extern Brush brush;

#endif //PP_BRUSH_H
