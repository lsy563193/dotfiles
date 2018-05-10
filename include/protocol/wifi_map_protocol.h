//
// Created by root on 4/10/18.
//

#ifndef PP_WIFI_MAP_H
#define PP_WIFI_MAP_H


#include <cstdint>
#include <map.h>

constexpr size_t WIFI_MAP_SIZE = 2;
constexpr float WIFI_MAP_RESOLUTION = 0.05;
constexpr int16_t WIFI_MAP_WIDTH_HALF = (WIFI_MAP_SIZE/WIFI_MAP_RESOLUTION/2);
constexpr size_t DATA_SIZE = ( 6 +WIFI_MAP_SIZE * WIFI_MAP_SIZE / WIFI_MAP_RESOLUTION);

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

//    void display();
	WifiMap app_map_{};
private:

    void runLengthEncoding(GridMap &grid_map, WifiMap &wifi_map, const BoundingBox2 &bound);


    int changeCost(int cost) ;

private:

//    Vector2<int> left_low_conner;
//    int hight;

//    uint8_t data[wifi_map->size()*3];
//#define (4 * 4 * 0.05) DATA_SIZE
    //uint8_t *data_;
    //size_t valid_size_{};
    boost::mutex data_mutex;
	WifiMap wifi_map_{};
	bool new_arrival_;
//    std::vector<uint8_t> data;
//    BoundingBox2 bound{};
//    std::shared_ptr<WifiMap> wifi_map = std::make_shared<WifiMap>();
};


extern WifiMapManage wifiMapManage;
#endif //PP_WIFI_MAP_H
