//
// Created by austin on 17-12-3.
//

#include "ros/ros.h"
#include "path_algorithm.h"

void APathAlgorithm::displayCellPath(const Cells &path)
{
	std::string     msg = __FUNCTION__;

	msg += " " + std::to_string(__LINE__) + ": Path size(" + std::to_string(path.size()) + "):";
	for (auto it = path.begin(); it != path.end(); ++it) {
		msg += "(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ")->";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

void APathAlgorithm::displayTargetList(const Cells &target_list)
{
	std::string     msg = __FUNCTION__;
	msg += " " + std::to_string(__LINE__) + ": Targers(" + std::to_string(target_list.size()) + "):";
	for (auto it = target_list.begin(); it != target_list.end(); ++it) {
		msg += "(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ", " + "),";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

void APathAlgorithm::displayPointPath(const Points &point_path)
{
	std::string     msg = __FUNCTION__;
	msg += " " + std::to_string(__LINE__) + ": Targers(" + std::to_string(point_path.size()) + "):";
	for (auto it = point_path.begin(); it != point_path.end(); ++it) {
		msg += "(" + std::to_string(it->X) + ", " + std::to_string(it->Y) + ", " + std::to_string(it->TH) + "),";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

void APathAlgorithm::optimizePath(GridMap &map, Cells& path)
{
	// Optimize only if the path have more than 3 cells.
	if (path.size() > 3) {
		ROS_INFO("%s %d: Start optimizing Path",__FUNCTION__,__LINE__);
		auto it = path.begin();
		for (uint16_t i = 0; i < path.size() - 3; i++) {
			ROS_DEBUG("%s %d: i: %d, size: %ld.", __FUNCTION__, __LINE__, i, path.size());
			auto it_ptr1 = it;

			auto it_ptr2 = it_ptr1;
			it_ptr2++;

			auto it_ptr3 = it_ptr2;
			it_ptr3++;

			// Just for protection, in case the iterator overflow. It should not happen under (i < path_points.size() - 3).
			if (it_ptr3 == path.end())
				break;

			if (it_ptr2->X == it_ptr3->X) {		// X coordinates are the same for p1, p2, find a better Y coordinate.
				int16_t x_min, x_max;
				x_min = x_max = it_ptr2->X;

				int16_t	ei, ej, si, sj;
				sj = it_ptr1->X > it_ptr2->X ? it_ptr2->X : it_ptr1->X;
				ej = it_ptr1->X > it_ptr2->X ? it_ptr1->X : it_ptr2->X;
				si = it_ptr2->Y > it_ptr3->Y ? it_ptr3->Y : it_ptr2->Y;
				ei = it_ptr2->Y > it_ptr3->Y ? it_ptr2->Y : it_ptr3->Y;

				bool blocked_min, blocked_max;
				blocked_min = blocked_max = false;
				while (!blocked_min || !blocked_max) {
					for (int16_t j = si; j <= ei && (!blocked_min || !blocked_max); j++) {
						if (!blocked_min && (x_min - 1 < sj ||
								map.getCell(COST_MAP, x_min - 1, j) == COST_HIGH)) {
							blocked_min = true;
						}
						if (!blocked_max && (x_max + 1 > ej ||
								map.getCell(COST_MAP, x_max + 1, j) == COST_HIGH)) {
							blocked_max = true;
						}
					}
					if (!blocked_min) {
						x_min--;
					}
					if (!blocked_max) {
						x_max++;
					}
				}

				if (it_ptr3->X != (x_min + x_max) / 2) {
					it_ptr2->X = it_ptr3->X = (x_min + x_max) / 2;
				}
				//ROS_INFO("%s %d: Loop i:%d\tx_min: %d\tx_max: %d\tget x:%d", __FUNCTION__, __LINE__, i, x_min, x_max, (x_min + x_max) / 2);
			} else {
				int16_t y_min, y_max;
				y_min = y_max = it_ptr2->Y;

				int16_t	ei, ej, si, sj;
				sj = it_ptr1->Y > it_ptr2->Y ? it_ptr2->Y : it_ptr1->Y;
				ej = it_ptr1->Y > it_ptr2->Y ? it_ptr1->Y : it_ptr2->Y;
				si = it_ptr2->X > it_ptr3->X ? it_ptr3->X : it_ptr2->X;
				ei = it_ptr2->X > it_ptr3->X ? it_ptr2->X : it_ptr3->X;

				bool blocked_min, blocked_max;
				blocked_min = blocked_max = false;
				while (!blocked_min || !blocked_max) {
					for (int16_t j = si; j <= ei && (!blocked_min || !blocked_max); j++) {
						if (!blocked_min && (y_min - 1 < sj ||
								map.getCell(COST_MAP, j, y_min - 1) == COST_HIGH)) {
							blocked_min = true;
						}
						if (!blocked_max && (y_max + 1 > ej ||
								map.getCell(COST_MAP, j, y_max + 1) == COST_HIGH)) {
							blocked_max = true;
						}
					}
					if (!blocked_min) {
						y_min--;
					}
					if (!blocked_max) {
						y_max++;
					}
				}

				if (it_ptr3->Y != (y_min + y_max) / 2) {
					it_ptr2->Y = it_ptr3->Y = (y_min + y_max) / 2;
				}
				//ROS_INFO("%s %d: Loop i:%d\ty_min: %d\ty_max: %d\tget y:%d", __FUNCTION__, __LINE__, i, y_min, y_max, (y_min + y_max) / 2);
			}

			it++;
		}
		displayCellPath(path);
	} else
		ROS_INFO("%s %d:Path too short(size: %ld), optimization terminated.", __FUNCTION__, __LINE__, path.size());

}
Points APathAlgorithm::cells_generate_points(Cells &path)
{
//	displayCellPath(path);
	Points targets{};
	for(auto it = path.begin(); it < path.end(); ++it) {
		Point32_t target {GridMap::cellToCount((*it).X),GridMap::cellToCount((*it).Y),0};
		auto it_next = it+1;
		if (it->X == it_next->X)
			target.TH = it->Y > it_next->Y ? MAP_NEG_Y : MAP_POS_Y;
		else
			target.TH = it->X > it_next->X ? MAP_NEG_X : MAP_POS_X;
		targets.push_back(target);
	}
//		ROS_INFO("path.back(%d,%d,%d)",path.back().X, path.back().Y, path.back().TH);

	targets.back().TH = (targets.end()-2)->TH;
//	ROS_INFO("%s %d: path.back(%d,%d,%d), path.front(%d,%d,%d)", __FUNCTION__, __LINE__,
//					 path.back().X, path.back().Y, path.back().TH, path.front().X, path.front().Y, path.front().TH);
	return targets;
}

bool APathAlgorithm::generateShortestPath(GridMap &map, const Point32_t &curr,const Point32_t &target, const MapDirection &last_dir, Points &plan_path) {
	auto path_cell = findShortestPath(map, GridMap::pointToCell(curr), GridMap::pointToCell(target),last_dir, false);

	plan_path = cells_generate_points(path_cell);
}

