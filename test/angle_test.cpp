//
// Created by lsy563193 on 17-3-7.
//
#include <obstacle_detector/SegmentObstacle.h>
#include <mathematics.h>
#include "ros/ros.h"

using namespace std;

int main(int argc, char *argv[])
{
  std::vector<Cell_t> passed_path;
  passed_path.clear();
  passed_path.push_back({1,1,1});
  passed_path.push_back({1,2,1});
  passed_path.push_back({2,1,1});
	passed_path.push_back({2,2,1});
	Cell_t curr{2,3,0};
	ROS_INFO("passed_path.size(%d)", passed_path.size());
	auto pos = std::find(passed_path.begin(), passed_path.end(), curr);
	if(pos == passed_path.end())
		passed_path.push_back(curr);

	for(const auto& cell: passed_path)
	{
		ROS_INFO("cell(%d, %d, %d)", cell.X,cell.Y,cell.TH);
	}
}