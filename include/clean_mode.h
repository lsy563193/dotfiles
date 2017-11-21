//
// Created by lsy563193 on 17-9-27.
//

#ifndef PP_CLEAN_MODE_H
#define PP_CLEAN_MODE_H

#include "regulator.h"
enum {
Clean_Mode_Idle = 1,
Clean_Mode_WallFollow,
Clean_Mode_RandomMode,
Clean_Mode_Charging,
Clean_Mode_Go_Charger,
Clean_Mode_Sleep,
Clean_Mode_Test,
Clean_Mode_Remote,
Clean_Mode_Spot,
Clean_Mode_Navigation,
Clean_Mode_Exploration
};

bool cm_is_follow_wall();
bool cm_is_navigation();
bool cm_is_exploration();
bool cm_is_spot();
bool cm_is_go_charger();
void cm_set(uint8_t mode);
uint8_t cm_get(void);

class CleanMode:public RegulatorBase
{
public:
//	CleanMode(const Cell_t& start_cell, const Cell_t& target, const PPTargetType& path);
//	~CleanMode();
	virtual void mark()=0;
	virtual bool isStop()=0;
	virtual bool isExit()=0;
	virtual bool isReach() =0;
	virtual bool isSwitch() = 0 ;
	virtual bool findTarget(Cell_t& curr);
//	virtual bool path_next()=0;
	void setTarget() {p_reg_->setTarget();}

	void display();

	bool find_target(Cell_t& curr);

	bool isMt(void) const
	{
		return p_reg_ == mt_reg_;
	}
	bool isBack(void) const
	{
		return p_reg_ == back_reg_;
	}
	bool isTurn(void) const
	{
		return p_reg_ == turn_reg_;
	}

	void adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	void setMt(void);

	virtual Cell_t updatePosition(const Point32_t &curr_point);

	Cell_t updatePath(const Cell_t& curr);
	void resetTriggeredValue(void);
	std::string getName()
	{
		std::string name = "CleanMode";
		return name;
	}

protected:
	RegulatorBase* p_reg_;
	RegulatorBase* mt_reg_;
	FollowWallRegulator* fw_reg_;
	LinearRegulator* line_reg_;
	GoToChargerRegulator* gtc_reg_;
	TurnRegulator* turn_reg_;
	BackRegulator* back_reg_;

	int32_t left_speed_;
	int32_t right_speed_;
	Cell_t last_;
};

class NavigationClean:public CleanMode{
public:
	NavigationClean(const Cell_t& start_cell, const Cell_t& target, const PPTargetType& path);
	~NavigationClean();
//	void adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	Cell_t updatePosition(const Point32_t &curr_point);
	void mark() {CleanMode::mark();};
	bool isStop();
	bool isExit();
	bool isReach();
	bool isSwitch();
	bool findTarget(Cell_t& curr);
//	bool path_next();

private:
};

class SpotClean:public CleanMode{
public:
	SpotClean(const Cell_t& start_cell, const Cell_t& target, const PPTargetType& path);
	~SpotClean();
//	void adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	void mark();
	bool isStop();
	bool isExit();
	bool isReach();
	bool isSwitch();

private:
};

class WallFollowClean:public CleanMode{
public:
	WallFollowClean(const Cell_t& start_cell, const Cell_t& target, const PPTargetType& path);
	~WallFollowClean();
//	void adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	void mark() {CleanMode::mark();};
	bool isStop();
	bool isExit();
	bool isReach();
	bool isSwitch();
	bool findTarget(Cell_t& curr);
	Cell_t updatePosition(const Point32_t &curr_point);
};

class Exploration:public CleanMode{
public:
	Exploration(const Cell_t& start_cell, const Cell_t& target, const PPTargetType& path);
	~Exploration();
//	void adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	void mark() {CleanMode::mark();};
	bool isStop();
	bool isExit();
	bool isReach();
	bool isSwitch();
	bool findTarget(Cell_t& curr);

private:
};

#endif //PP_CLEAN_MODE_H
