//
// Created by root on 11/17/17.
//

#ifndef PP_BRUSH_H
#define PP_BRUSH_H

class Brush {
public:
	Brush(void);

	void stop(void);

	void setMainPwm(uint16_t PWM);

	void setSidePwm(uint16_t L, uint16_t R);

	void setLeftPwm(uint16_t L);

	void setRightPwm(uint16_t R);

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

	uint8_t oc_left_cnt_;
	uint8_t oc_main_cnt_;
	uint8_t oc_right_cnt_;
private:

//Value for saving SideBrush_PWM
	uint16_t left_pwm_;
	uint16_t right_pwm_;
	uint16_t main_pwm_;

	bool is_left_oc_;
	bool is_right_oc_;
	bool is_main_oc_;

	bool reset_left_oc_;
	bool reset_right_oc_;
	bool reset_main_oc_;
};
extern Brush brush;

#endif //PP_BRUSH_H
