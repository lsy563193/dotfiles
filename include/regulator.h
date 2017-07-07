//
// Created by lsy563193 on 6/28/17.
//

#ifndef PP_REGULATOR_BASE_H
#define PP_REGULATOR_BASE_H


class RegulatorBase {
public:
	bool isExit(){
		return isStop() || isReach();
	};
	virtual bool isSwitch() = 0;
	virtual bool adjustSpeed(int32_t&, int32_t&)=0;
	virtual bool isReach() = 0;
//	RegulatorType getType(){return type_;};
private:
	bool isStop();

public:
	static Point32_t s_target;
	static Point32_t s_origin;
};

class BackRegulator: public RegulatorBase{
public:
	BackRegulator();
	bool adjustSpeed(int32_t&, int32_t&);
	bool isSwitch();
	bool isReach();
	void setOrigin(){
		auto pos_x = robot::instance()->getOdomPositionX();
		auto pos_y = robot::instance()->getOdomPositionY();
		pos_x_ = pos_x;
		pos_y_ = pos_y;
	}

private:
	float pos_x_,pos_y_;
	int counter_;
	int32_t speed_;
};

class TurnRegulator: public RegulatorBase{
public:

	TurnRegulator(uint16_t speed_max);
	bool adjustSpeed(int32_t&, int32_t&);
	bool isSwitch();
	bool isReach();
	void setTarget(int16_t target_angle){
		target_angle_ = target_angle;
	};

private:
	int16_t target_angle_;
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
	SelfCheckRegulator(){};
	~SelfCheckRegulator(){
		set_wheel_speed(0, 0);
	};
	bool adjustSpeed(uint8_t bumper_jam_state);
};

class FollowWallRegulator:public RegulatorBase{

public:
	FollowWallRegulator(CMMoveType type);
	~FollowWallRegulator(){ set_wheel_speed(0,0); };
	bool adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	bool isReach();
	bool isSwitch();
//	void setTarget(Point32_t target){s_target = target;};
//	void setOrigin(Point32_t origin){s_origin = origin;};

private:
	int32_t	 previous_;
	CMMoveType type_;
	int jam_;
};

class LinearSpeedRegulator{
public:
	LinearSpeedRegulator(int32_t max):speed_max_(max),integrated_(0),base_speed_(BASE_SPEED),integration_cycle_(0),tick_(0),turn_speed_(4){};
	~LinearSpeedRegulator(){
		set_wheel_speed(0, 0);
	};
	bool adjustSpeed(Point32_t Target, bool slow_down, bool &rotate_is_needed);

private:
	int32_t speed_max_;
	int32_t integrated_;
	int32_t base_speed_;
	uint8_t integration_cycle_;
	uint32_t tick_;
	uint8_t turn_speed_;
};

class RegulatorProxy:public RegulatorBase{
public:
	RegulatorProxy(Point32_t origin, Point32_t target);
	~RegulatorProxy();
	void switchToNext();
	bool adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	bool isSwitch();
	bool isReach();
	RegulatorBase* getType(){ return p_reg_; };
private:
	RegulatorBase* p_reg_;
	TurnRegulator* turn_reg_;
	BackRegulator* back_reg_;
	FollowWallRegulator* follow_wall_reg_;

};

#endif //PP_REGULATOR_BASE_H
