//
// Created by austin on 17-12-3.
//

#ifndef PP_ACTION_OPEN_GYRO_HPP
#define PP_ACTION_OPEN_GYRO_HPP

#include "action_new.hpp"

class OpenGyroAction: public Action{
public:
	OpenGyroAction();
	~OpenGyroAction() = default;
	void action();

	bool isFinishAndUpdatePlan(){};
	bool isFinishAndUpdateAction();

	Action* getNextAction(){};
};

class NavOpenGyroAction: public OpenGyroAction{
public:
	NavOpenGyroAction();
	~NavOpenGyroAction();

	Action* getNextAction();

};
#endif //PP_ACTION_OPEN_GYRO_HPP
