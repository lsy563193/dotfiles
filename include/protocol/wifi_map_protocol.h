//
// Created by root on 4/10/18.
//

#ifndef PP_WIFI_MAP_H
#define PP_WIFI_MAP_H


#include <cstdint>
#include <map.h>


typedef std::pair<uint8_t, uint8_t > MapElem;
typedef std::tuple<Cell_t, int16_t ,std::vector<MapElem>>  WifiMap;
//typedef std::tuple<Vector2, int, > wifi_protocol
class WifiMapManage {
public:
	WifiMapManage();
//    WifiMapManage() = delete;

//    WifiMapManage(GridMap& grid_map);

    WifiMap* getData();

    void serialize(GridMap& grid_map,const BoundingBox2& bound);

    void runLengthEncoding(GridMap &grid_map, WifiMap &wifi_map, const BoundingBox2 &bound);
//    void display();
	WifiMap app_map_{};

	int convert(float resulution);
private:

    int changeCost(int cost) ;
private:

	boost::mutex data_mutex;
	WifiMap wifi_map_{};
	bool new_arrival_;
	float width_;
	float resulution_;
	int16_t width_half_;
	size_t size_;
	Point_t last_point_;
};


extern WifiMapManage wifiMapManage;
#endif //PP_WIFI_MAP_H
