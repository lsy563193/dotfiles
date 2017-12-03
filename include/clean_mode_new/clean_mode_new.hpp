//
// Created by austin on 17-12-3.
//

#ifndef PP_CLEAN_MODE_NEW_HPP
#define PP_CLEAN_MODE_NEW_HPP

#include "path_algorithm/path_algorithm.h"
#include "action_new/action_new.hpp"

class CleanModeNew{
public:
	CleanModeNew* run();
	virtual bool eventHandle(CleanModeNew** p_sp_next_clean_mode_, PathAlgorithm** p_sp_path_, Action** p_sp_action_){return false;};
	virtual bool updatePath(CleanModeNew** p_sp_next_clean_mode_, PathAlgorithm** p_sp_path_, Action** p_sp_action_){return false;};
	virtual bool updateAction(CleanModeNew** p_sp_next_clean_mode_, PathAlgorithm** p_sp_path_, Action** p_sp_action_){return false;};

protected:
	CleanModeNew* sp_next_clean_mode_;
	Action* sp_action_;
	PathAlgorithm* sp_path_algorithm_;
};

#endif //PP_CLEAN_MODE_NEW_HPP
