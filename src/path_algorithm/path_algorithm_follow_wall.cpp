//
// Created by lsy563193 on 12/13/17.
//

#include "ros/ros.h"
#include "path_algorithm.h"
#include "pp.h"

bool WFCleanPathAlgorithm::generatePath(GridMap &map, const Point32_t &curr, const MapDirection &last_dir, Points &targets)
{
	if (targets.empty()) {//fw ->linear
		auto curr = getPosition();
		curr.th = 0;
		targets.push_back(curr);
//		if (reach_cleaned_count == 0 ||
//				(reach_cleaned_count < ISOLATE_COUNT_LIMIT && !fw_is_time_up()/*get_work_time() < WALL_FOLLOW_TIME*/ &&
//				 wf_is_isolate())) {
			fw_map.reset(CLEAN_MAP);
			auto angle = 0;
//			if (reach_cleaned_count == -900) {
//				angle = -900;
//			}
			auto point = getPosition();
			point.th = ranged_angle(curr.th + angle);
			point = point.getRelative(8 * 1000, 0);
			targets.push_back(point);
		ROS_INFO("%s,%d: path_next_linear(%d)", __FUNCTION__, __LINE__,point.th);
//		}
		return true;
	}
	else//linear->fw
	{
		targets.clear();
		ROS_INFO("%s,%d: path_next_fw", __FUNCTION__, __LINE__);
		targets.push_back(getPosition());
		return true;
	}
}
