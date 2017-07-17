//
// Created by lsy563193 on 6/28/17.
//

#ifndef PP_REGULATOR_BASE_H
#define PP_REGULATOR_BASE_H

#include <move_type.h>

class RegulatorBase {
public:

	bool isExit();

	bool isStop(){
		return RegulatorBase::_isStop() || _isStop();
	};

	virtual bool isSwitch() = 0;
	virtual void adjustSpeed(int32_t&, int32_t&)=0;
	virtual bool _isStop()=0;
	virtual void setTarget() = 0;
	virtual bool isReach() = 0;

public:
	static Point32_t s_target;
	static Point32_t s_origin;
	static int16_t s_target_angle;
	static float s_pos_x;
	static float s_pos_y;
};

class BackRegulator: public RegulatorBase{
public:
	BackRegulator();
	void adjustSpeed(int32_t&, int32_t&);
	bool isSwitch();
	bool _isStop();
	void setTarget(){
		s_pos_x = robot::instance()->getOdomPositionX();
		s_pos_y = robot::instance()->getOdomPositionY();
	}

protected:
	bool isReach();

private:
	int counter_;
	int32_t speed_;
};

class TurnRegulator: public RegulatorBase{
public:

	TurnRegulator(int16_t angle);
	void adjustSpeed(int32_t&, int32_t&);
	bool isSwitch();
	bool _isStop();
	void setTarget();

protected:
	bool isReach();

private:
	uint16_t accurate_;
	uint16_t speed_max_;
};

class TurnSpeedRegulator{
public:
	TurnSpeedRegulator(uint8_t speed_max,uint8_t speed_min, uint8_t speed_start):
					speed_max_(speed_max),speed_min_(speed_min), speed_(speed_start),tick_(0){};
	~TurnSpeedRegulator() {
		set_wheel_speed(0, 0);
	}
	bool adjustSpeed(int16_t diff, uint8_t& speed_up);

private:
	uint8_t speed_max_;
	uint8_t speed_min_;
	uint8_t speed_;
	uint32_t tick_;
};

class SelfCheckRegulator{
public:
	SelfCheckRegulator(){
		ROS_WARN("%s, %d: ", __FUNCTION__, __LINE__);
	};
	~SelfCheckRegulator(){
		set_wheel_speed(0, 0);
	};
	void adjustSpeed(uint8_t bumper_jam_state);
};

class FollowWallRegulator:public RegulatorBase{

public:
	FollowWallRegulator(Point32_t origin, Point32_t target);
	~FollowWallRegulator(){ set_wheel_speed(0,0); };
	void adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	bool isSwitch();
	bool _isStop();
	void setTarget(){ };
protected:
	bool isReach();

private:
	int32_t	 previous_;
	int jam_;
};

class LinearRegulator: public RegulatorBase{
public:
	LinearRegulator(Point32_t);
	~LinearRegulator(){ };
	bool _isStop();
	bool isSwitch();
	void adjustSpeed(int32_t&, int32_t&);
	void setTarget() { };

protected:
	bool isReach();

private:
	int32_t speed_max_;
	int32_t integrated_;
	int32_t base_speed_;
	uint8_t integration_cycle_;
	uint32_t tick_;
	uint8_t turn_speed_;
};

class RegulatorManage:public RegulatorBase{
public:
	RegulatorManage(Point32_t origin, Point32_t target);
	~RegulatorManage();
	void adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	bool isSwitch();
	bool _isStop();
	bool isReach();
	void mark();
	void setTarget() {p_reg_->setTarget();}

	void switchToNext();
private:
	RegulatorBase* p_reg_;
	RegulatorBase* mt_reg_;
	TurnRegulator* turn_reg_;
	BackRegulator* back_reg_;

};

#endif //PP_REGULATOR_BASE_H
