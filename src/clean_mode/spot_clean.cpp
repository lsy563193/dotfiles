//
// Created by lsy563193 on 11/29/17.
//
#include "pp.h"

//SpotClean
SpotClean::SpotClean(const Cell_t& curr, const Cell_t& target_cell, const PPTargetType& path)
{
	g_plan_path.clear();
	s_curr_p = {cost_map.get_x_count(),cost_map.get_y_count()};
	auto target = cost_map.cell_to_point(target_cell);

	back_reg_ = new BackMovement();
	fw_reg_ = new FollowWallMovement(s_curr_p, target);
	line_reg_ = new ForwardMovement(target, path);
	gtc_reg_ = new GoToChargerMovement();
	turn_reg_ = new TurnMovement(0);
	mt_reg_ = line_reg_;

	p_reg_ = mt_reg_;


	ROS_INFO("%s, %d: SpotClean finish", __FUNCTION__, __LINE__);
	g_check_path_in_advance = false;

	g_passed_path.clear();
	g_passed_path.push_back(curr);

}

SpotClean::~SpotClean()
{
	delete turn_reg_;
	delete back_reg_;
	delete line_reg_;
	delete fw_reg_;
	delete gtc_reg_;
}

void SpotClean::mark()
{
	CleanMode::mark();

	if (ev.rcon_triggered || ev.obs_triggered || ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered)
	{
		SpotType spt = SpotMovement::instance()->getSpotType();
		if (spt == CLEAN_SPOT || spt == NORMAL_SPOT)
			SpotMovement::instance()->setOBSTrigger();
	}
}

bool SpotClean::isReach()
{
	if (mt.is_linear()) // Robot is cleaning current line.
	{
		if (isMt())
		{
			return line_reg_->isCellReach(); // For reaching target.
		}
		else if (isBack())
			return back_reg_->isReach();
	}

	return false;
}

bool SpotClean::isExit()
{
	if(CleanMode::isExit())
		return true;

	return false;
}

bool SpotClean::isStop()
{
	if(CleanMode::isStop())
		return true;

	else if (cs.is_clean())
	{
		if (mt.is_linear()) {
			if (isMt())
				return (line_reg_->isRconStop() || line_reg_->isOBSStop()
								|| line_reg_->isLidarStop() || line_reg_->isBoundaryStop());
			else if (isBack())
				return back_reg_->isLidarStop();
		}
	}
	return false;
}

bool SpotClean::isSwitch()
{
	auto old_p_reg_ = p_reg_;

	if (cs.is_clean()) {
		if (mt.is_linear()) // Robot is cleaning current line.
		{
			if (isTurn()) {
				if (turn_reg_->isReach())
					p_reg_ = mt_reg_;
				else if (turn_reg_->shouldMoveBack())
					p_reg_ = back_reg_;
			}
			else if (isMt() && line_reg_->shouldMoveBack())
				p_reg_ = back_reg_;
		}
	}

	if (old_p_reg_ != p_reg_)
	{
		ROS_INFO("%s %d: Switch from %s to %s.", __FUNCTION__, __LINE__, (old_p_reg_->getName()).c_str(), (p_reg_->getName()).c_str());
		setTarget();
		return true;
	}

	return false;
}
