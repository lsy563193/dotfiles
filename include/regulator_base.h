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

//	enum RegulatorType{
//		RegBase,
//		RegBack,
//		RegTurn,
//		RegFlwl,
//	};
//	int type_;
};

class FollowWallRegulator:public RegulatorBase{

public:
	FollowWallRegulator(CMMoveType type);
	~FollowWallRegulator(){ set_wheel_speed(0,0); };
	bool adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	bool isReach();
	bool isSwitch();
	void setTarget(Point32_t target){target_ = target;};
	void setOrigin(Point32_t origin){origin_ = origin;};

private:
	int32_t	 previous_;
	CMMoveType type_;
	Point32_t target_;
	Point32_t origin_;
	int jam_;
};

class BackRegulator: public RegulatorBase{
public:
	BackRegulator();
	bool adjustSpeed(int32_t&, int32_t&);
	bool isSwitch();
	bool isReach();
	void setOrigin(float pos_x, float pos_y){
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


class RegulatorProxy:public RegulatorBase{
public:
	RegulatorProxy(RegulatorBase* p_reg);
	void switchTo(RegulatorBase *p_reg){p_reg_ = p_reg;};
	bool adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	bool isSwitch();
	bool isReach();
	RegulatorBase* getType(){ return p_reg_; };
private:
	RegulatorBase* p_reg_;
	uint8_t bumper_;

};

#endif //PP_REGULATOR_BASE_H