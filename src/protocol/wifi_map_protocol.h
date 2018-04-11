//
// Created by root on 4/10/18.
//

#ifndef PP_WIFI_MAP_H
#define PP_WIFI_MAP_H


#include <cstdint>
#include <map.h>

constexpr size_t WIFI_MAP_SIZE = 4;
constexpr float WIFI_MAP_RESOLUTION = 0.05;
constexpr uint16_t WIFI_MAP_WIDTH_HALF = (WIFI_MAP_SIZE/WIFI_MAP_RESOLUTION/2);
constexpr size_t DATA_SIZE = (WIFI_MAP_SIZE * WIFI_MAP_SIZE / WIFI_MAP_RESOLUTION);

typedef std::pair<uint8_t, uint16_t > MapElem;
typedef std::tuple<Cell_t, int16_t ,std::vector<MapElem>>  WifiMap;
//typedef std::tuple<Vector2, int, > wifi_protocol
class WifiMapManage {
public:
//    WifiMapManage() = delete;

//    WifiMapManage(GridMap& grid_map);

    void getData(uint8_t* data, size_t* size);

    void serialize(GridMap& grid_map);

//    void display();
private:

    void cursorCompression(GridMap& grid_map, WifiMap& wifi_map);


    uint8_t changeCost(int cost) ;

private:

//    Vector2<int> left_low_conner;
//    int hight;

//    uint8_t data[wifi_map->size()*3];
//#define (4 * 4 * 0.05) DATA_SIZE
    uint8_t data_[DATA_SIZE];
    size_t valid_size_{};
    boost::mutex data_mutex;
//    std::vector<uint8_t> data;
//    BoundingBox2 bound{};
//    std::shared_ptr<WifiMap> wifi_map = std::make_shared<WifiMap>();
};


extern WifiMapManage wifiMapManage;
#endif //PP_WIFI_MAP_H
