//
// Created by austin on 17-12-3.
//

#ifndef PP_CLEAN_MODE_NAVIGATION_NEW_HPP
#define PP_CLEAN_MODE_NAVIGATION_NEW_HPP

#include "clean_mode_new/clean_mode_new.hpp"

class NavigationModeNew:public CleanModeNew{
public:
	NavigationModeNew();
	~NavigationModeNew(){};

	bool eventHandle(CleanModeNew** p_sp_next_clean_mode_, PathAlgorithm** p_sp_path_, Action** p_sp_action_);
	bool updatePath(CleanModeNew** p_sp_next_clean_mode_, PathAlgorithm** p_sp_path_, Action** p_sp_action_);
	bool updateAction(CleanModeNew** p_sp_next_clean_mode_, PathAlgorithm** p_sp_path_, Action** p_sp_action_);
};

#endif //PP_CLEAN_MODE_NAVIGATION_NEW_HPP
