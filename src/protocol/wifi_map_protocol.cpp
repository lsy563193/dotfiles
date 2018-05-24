//
// Created by root on 4/10/18.
//

#include "protocol/wifi_map_protocol.h"
#include "ros/ros.h"
#include "robot.hpp"

WifiMapManage wifiMapManage;

WifiMapManage::WifiMapManage()
{
	new_arrival_ = false;
	width_ = 2.0;
	resulution_= 0.05;
	width_half_ = ((width_/resulution_/2));
	//size_ = ( 6 +(width_ * width_)/ resulution_;
	last_point_ = {0,0,0};

}

void WifiMapManage::runLengthEncoding(GridMap &grid_map, WifiMap &wifi_map, const BoundingBox2 &bound)
{
	std::get<0>(wifi_map) = bound.max;
	std::get<1>(wifi_map) = (bound.max.x - bound.min.x);
	auto& data = std::get<2>(wifi_map);
	data.clear();
	int last_cost=50;//init
	int size=0;
	bool first_time = true;
	// -- loop through left top corner to right down corner
	for(auto j= bound.max.x; j> bound.min.x; j--)
	{
		for(auto i= bound.max.y; i> bound.min.y; i--)
		{
			auto cost = grid_map.getCell(CLEAN_MAP,j,i);
			auto it_cost = changeCost(cost);
			if(first_time)
			{
				first_time = false;
				last_cost = it_cost;
			}
			if(it_cost != last_cost)
			{
				data.push_back({last_cost, size});
				last_cost = it_cost;
				size = 1;
				continue;
			}
			size++;
			if(size >=255)
			{
				data.push_back({last_cost, size});
				size = 0;
			}
		}
	}
	if(size>0)
		data.push_back({last_cost, size});

	ROS_INFO("%s %d: End run-length encoding, data type number(%d).", __FUNCTION__, __LINE__, data.size());
}

int WifiMapManage::changeCost(int cost)
{
	if(cost == 10)//block
		return 0x01;
	else if(cost == 1)//clean
		return 0x02;
	else if(cost == 0)//unclean
		return 0x03;
	else
		return 0x03;
}

//void WifiMapManage::display() {
//    ROS_INFO("display wifi map data:");
////	boost::mutex::scoped_lock lock(data_mutex);
////	std::copy(std::begin(data_), std::end(data_), std::ostream_iterator<int>(std::cout, ","));
////	printf("\n");
//}

void WifiMapManage::serialize(GridMap& grid_map, const BoundingBox2& bound)
{
	boost::mutex::scoped_lock lock(data_mutex);
	runLengthEncoding(grid_map, wifi_map_, bound);
	new_arrival_ = true;
}

WifiMap *WifiMapManage::getData()
{
	boost::mutex::scoped_lock lock(data_mutex);
	if(new_arrival_)
	{
		new_arrival_ = false;
		app_map_= wifi_map_;
		//auto& data = std::get<2>(wifi_map_);
		//data.clear();
		return &app_map_;
	}
	else
		return nullptr;
}

int WifiMapManage::convert(float slam_resulution)
{
	auto dist = last_point_.Distance(getPosition());
	last_point_ = getPosition();
	width_ = (dist>=1.8)? 4.0: 2.0;
	ROS_INFO("%s,%d,distance \033[1;31m%f\033[0m",__FUNCTION__,__LINE__,dist);
	width_half_ = ((width_/resulution_/2));

	GridMap app_grid_map;
	Cell_t cell{static_cast<int16_t>(getPosition().x * resulution_/ slam_resulution),
				static_cast<int16_t>(getPosition().y * resulution_/ slam_resulution)};
	auto size = width_half_;
	BoundingBox2 bound = {{static_cast<int16_t >(-size), static_cast<int16_t >(-size)},{size,size}};
	cell = {static_cast<int16_t>(round(last_point_.x / 0.05)), static_cast<int16_t>(round(last_point_.y / 0.05))};
	// -- left top corner (max)
	bound.SetMaximum(cell + Cell_t{width_half_, width_half_});
	// -- right down corner (min)
	bound.SetMinimum(cell - Cell_t{width_half_, width_half_});
	ROS_INFO("\033[1;35m%s %d: size(%d), curr((%d,%d),(%d,%d)),bound({%d,%d}{%d,%d})\033[0m"
	,__FUNCTION__, __LINE__, size,cell.x, cell.y, getPosition().toCell().x, getPosition().toCell().y, bound.min.x, bound.min.y,bound.max.x, bound.max.y);
	app_grid_map.convertFromSlamMap(resulution_, 0.25, bound);
	serialize(app_grid_map, bound);
	return 0;
}
