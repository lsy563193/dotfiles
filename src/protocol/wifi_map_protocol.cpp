//
// Created by root on 4/10/18.
//

#include "protocol/wifi_map_protocol.h"

WifiMapManage wifiMapManage;

WifiMapManage::WifiMapManage()
{
	new_arrival_ = false;
}

void WifiMapManage::runLengthEncoding(GridMap &grid_map, WifiMap &wifi_map, const BoundingBox2 &bound)
{
//	bound = grid_map.generateBound();
	//ROS_INFO("%s %d: Begin run-length encoding.", __FUNCTION__, __LINE__);
	std::get<0>(wifi_map) = bound.min;
	std::get<1>(wifi_map) = (bound.max.x - bound.min.x)+1;
	printf("\033[32m wifi map width %d \033[0m\n",(bound.max.x - bound.min.x)+1);
	auto& data = std::get<2>(wifi_map);
	int last_cost=50;//init
	int size=0;
	bool first_time = true;
	for(auto j= bound.min.y; j<= bound.max.y; j++)
	{
		for(auto i= bound.min.x; i<= bound.max.x; i++)
		{
			auto cost = grid_map.getCell(CLEAN_MAP,i,j);
			auto it_cost = changeCost(cost);
			/*
			if(it_cost == 2)
				printf("\033[33m%d\033[0m",it_cost);
			else if(it_cost == 1)
				printf("\033[32m%d\033[0m",it_cost);
			else if(it_cost == 3)
				printf("\033[35m%d\033[0m",it_cost);
			*/
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
		//printf("\n");
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

    //left_low_conner_ = std::get<0>(wifi_map);
	//width_ = std::get<1>(wifi_map);
	//map_data_ = std::get<2>(wifi_map);
	/*
	valid_size_ = 6 + map_data.size()*3;
	ROS_ASSERT(valid_size_<10000);

	ROS_INFO("%s %d: valid_size_ %d, DATA_SIZE %d ", __FUNCTION__, __LINE__, 6 + valid_size_, DATA_SIZE);
	{
		memset(data_, 0, DATA_SIZE);

		*(data_+0)= static_cast<uint8_t>(left_low_conner.x);
		*(data_+1) = static_cast<uint8_t>(left_low_conner.x >> 8);
		*(data_+2)= static_cast<uint8_t>(left_low_conner.y);
		*(data_+3) = static_cast<uint8_t>(left_low_conner.y >> 8);
		*(data_+4)= static_cast<uint8_t>(width);
		*(data_+5) = static_cast<uint8_t>(width >> 8);

		for (size_t i = 0; i <= map_data.size(); ++i)
		{
			*(data_+(6 + i * 3))= map_data[i].first;
			*(data_+(6 + i * 3 + 1))= static_cast<uint8_t >(map_data[i].second);
			*(data_+(6 + i * 3 + 2))= static_cast<uint8_t >(map_data[i].second >> 8);
		}
		ROS_INFO("display wifi map : left_low_conner(%d,%d),width(%d),h(%d):", left_low_conner.x, left_low_conner.y,
				 width, map_data.size());
		*/
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
	}
		 */
}

WifiMap *WifiMapManage::getData()
{

	boost::mutex::scoped_lock lock(data_mutex);
	if(new_arrival_)
	{
		new_arrival_ = false;
		app_map_= wifi_map_;
		auto& data = std::get<2>(wifi_map_);
		data.clear();
		return &app_map_;
	}
	else
		return nullptr;
}
