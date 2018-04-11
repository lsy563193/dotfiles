//
// Created by root on 4/10/18.
//

#include "wifi_map_protocol.h"

WifiMapManage wifiMapManage;

void WifiMapManage::cursorCompression(GridMap& grid_map, WifiMap& wifi_map) {
	auto bound = grid_map.generateBound();
	std::get<0>(wifi_map) = bound.min;
	std::get<1>(wifi_map) = bound.max.x - bound.min.x;
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
}

uint8_t WifiMapManage::changeCost(int cost) {
    {
//	if(cost == 100)
//		return 2;
//    else if(cost == -1)
//		return 1;
//	else//cost = 0
//		return 0;
//    if(cost != 0 || cost != 1)
//		return 2;
        return cost;
    }
}

//void WifiMapManage::display() {
//    ROS_INFO("display wifi map data:");
////	boost::mutex::scoped_lock lock(data_mutex);
////	std::copy(std::begin(data_), std::end(data_), std::ostream_iterator<int>(std::cout, ","));
////	printf("\n");
//}

void WifiMapManage::serialize(GridMap& grid_map) {

	WifiMap wifi_map;
    cursorCompression(grid_map, wifi_map);
    auto left_low_conner = std::get<0>(wifi_map);
	auto width = std::get<1>(wifi_map);
	auto& map_data = std::get<2>(wifi_map);

	{
		boost::mutex::scoped_lock lock(data_mutex);
		memset(data_, 0, DATA_SIZE);
		data_[0] = left_low_conner.x;
		data_[1] = left_low_conner.x >> 8;
		data_[2] = left_low_conner.y;
		data_[3] = left_low_conner.y >> 8;
		data_[4] = static_cast<uint8_t>(width);
		data_[5] = static_cast<uint8_t>(width >> 8);
        size_t i = 0;
		for (; i < map_data.size(); ++i) {
			data_[6 + i * 3] = map_data[i].first;
			data_[6 + i * 3 + 1] = static_cast<uint8_t >(map_data[i].second);
			data_[6 + i * 3 + 2] = static_cast<uint8_t >(map_data[i].second >> 8);
		}
        valid_size_ = 6 + i*3;

		//display
		for(auto y = left_low_conner.x; y< DATA_SIZE/width; y++) {
			for (auto x = left_low_conner.x; x < left_low_conner.x + width; x++) {
                printf("%d", data_[y+x*width]);
			}
			printf("\n");
		}
	}
}

void WifiMapManage::getData(uint8_t* data, size_t* size) {

	boost::mutex::scoped_lock lock(data_mutex);
	data = data_;
	*size = valid_size_;
}

