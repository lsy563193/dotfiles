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
	void operate();
	void stop(void);
	void mainBrushResume();

	bool isOn()
	{
		return brush_status_ != brush_stop;
	}

	void setPWM(uint8_t L, uint8_t R, uint8_t M);
	void setLeftBrushPWM(uint8_t PWM);
	void setRightBrushPWM(uint8_t PWM);
	void setMainBrushPWM(uint8_t PWM);

	bool checkLeftBrushTwined();
	bool checkRightBrushTwined();

	bool checkBrushTwined(uint8_t brush_indicator);

	void setLeftOc(bool val)
	{
		side_brush_oc_status_[left] = val;
	}

	bool getLeftOc() const
	{
		return side_brush_oc_status_[left];
	}

	void setRightOc(bool val)
	{
		side_brush_oc_status_[right] = val;
	}

	bool getRightOc() const
	{
		return side_brush_oc_status_[right];
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

	// For consumable.
	void updateSideBrushTime(uint32_t addition_time);
	void updateMainBrushTime(uint32_t addition_time);
	uint32_t getSideBrushTime()
	{
		return side_brush_operation_time_;
	}
	uint32_t getMainBrushTime()
	{
		return main_brush_operation_time_;
	}
	void resetSideBurshTime();
	void resetMainBrushTime();

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

	bool is_main_oc_;

	uint16_t left_current_;
	uint16_t right_current_;
	uint16_t main_current_;

	enum {
		left = 0,
		right = 1,
	};
	bool side_brush_oc_status_[2]{false, false};
	uint8_t resume_stage_[2]{0, 0};
	double resume_start_time_[2]{0, 0};
	uint8_t resume_count_[2]{0, 0};

	// For consumable situation.
	uint32_t side_brush_operation_time_{0};
	uint32_t main_brush_operation_time_{0};

};
extern Brush brush;

#endif //PP_BRUSH_H
