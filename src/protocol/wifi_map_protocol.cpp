//
// Created by root on 4/10/18.
//

#include "protocol/wifi_map_protocol.h"

WifiMapManage wifiMapManage;

void WifiMapManage::runLengthEncoding(GridMap &grid_map, WifiMap &wifi_map, const BoundingBox2 &bound){
//	bound = grid_map.generateBound();
	ROS_INFO("%s %d: Begin run-length encoding.", __FUNCTION__, __LINE__);
	std::get<0>(wifi_map) = bound.min;
	std::get<1>(wifi_map) = bound.max.x - bound.min.x+1;
	auto& data = std::get<2>(wifi_map);
	for(auto i= bound.min.x; i<= bound.max.x; i++)
	{
		int curr_cost=250;//init
		int size=0;
		for(auto j= bound.min.y; j<= bound.max.y; j++)
		{
			auto it_cost = grid_map.getCell(CLEAN_MAP, i , j);
			if(it_cost != curr_cost)
			{
				if(size != 0)//init don't push
				{
					data.push_back({changeCost(curr_cost), size});
					size = 0;
				}
				curr_cost = it_cost;
			}
			size++;
		}
		data.push_back({changeCost(curr_cost), size});
	}
	ROS_INFO("%s %d: End run-length encoding, data size(%d).", __FUNCTION__, __LINE__, data.size());
}

uint8_t WifiMapManage::changeCost(int cost)
{
	if(cost == 100)//block
		return 0x01;
	else if(cost == 0 || cost == 1)//clean
		return 0x02;
	else if(cost == -1)//unclean
		return 0x03;
}

//void WifiMapManage::display() {
//    ROS_INFO("display wifi map data:");
////	boost::mutex::scoped_lock lock(data_mutex);
////	std::copy(std::begin(data_), std::end(data_), std::ostream_iterator<int>(std::cout, ","));
////	printf("\n");
//}

void WifiMapManage::serialize(GridMap& grid_map, const BoundingBox2& bound) {

	WifiMap wifi_map{};
	runLengthEncoding(grid_map, wifi_map, bound);
    auto left_low_conner = std::get<0>(wifi_map);
	auto width = std::get<1>(wifi_map);
	auto& map_data = std::get<2>(wifi_map);
	valid_size_ = 6 + map_data.size()*3;
	ROS_ASSERT(valid_size_<10000);

	ROS_INFO("%s %d: valid_size_ %d, DATA_SIZE %d ", __FUNCTION__, __LINE__, 6 + valid_size_, DATA_SIZE);
	{
		boost::mutex::scoped_lock lock(data_mutex);
		memset(data_, 0, DATA_SIZE);
		data_[0] = static_cast<uint8_t>(left_low_conner.x);
		data_[1] = static_cast<uint8_t>(left_low_conner.x >> 8);
		data_[2] = static_cast<uint8_t>(left_low_conner.y);
		data_[3] = static_cast<uint8_t>(left_low_conner.y >> 8);
		data_[4] = static_cast<uint8_t>(width);
		data_[5] = static_cast<uint8_t>(width >> 8);
		for (size_t i = 0; i <= map_data.size(); ++i)
		{
			data_[6 + i * 3] = map_data[i].first;
			data_[6 + i * 3 + 1] = static_cast<uint8_t >(map_data[i].second);
			data_[6 + i * 3 + 2] = static_cast<uint8_t >(map_data[i].second >> 8);
		}
		ROS_INFO("display wifi map : left_low_conner(%d,%d),width(%d),h(%d):", left_low_conner.x, left_low_conner.y,
				 width, map_data.size());
		/*
//		std::ostringstream outString;
		auto count =0;
		auto line =0;
		printf("%40d", bound.min.x);
		for(auto c = 6; c < valid_size_; c++) {
//			outString << data_[c];
			printf("%d", data_[c]);
			if ((c - 6) % 3 == 0)
			{
//				outString << ' ';
				printf(" ");
			}
			else if ((c - 6) % 3 == 1) {
//				outString << ',';
				count += data_[c];
				printf(",");
			}
			else if ((c - 6) % 3 == 2)
			{
//				outString << '|';
				if(count >= width)
				{
					line++;
					printf("\n%4d:\t", bound.min.x+line);
					count = 0;
				}
				else
					printf("|");
			}
		}
//        printf("%s", outString.str().c_str());
		printf("\n");
		 */
	}
}

void WifiMapManage::getData(uint8_t* data, size_t* size) {

	boost::mutex::scoped_lock lock(data_mutex);
	data = data_;
	*size = valid_size_;
}

