//
// Created by austin on 17-12-1.
//

#ifndef PP_SLAM_MAP_H
#define PP_SLAM_MAP_H

#include <cstdint>
#include <vector>
#include <boost/thread/mutex.hpp>

class SlamMap
{
public:
	SlamMap(){};
	~SlamMap(){};

	void setWidth(uint32_t width)
	{
		width_ = width;
	}

	uint32_t getWidth()
	{
		return width_;
	}

	void setHeight(uint32_t height)
	{
		height_ = height;
	}

	uint32_t getHeight()
	{
		return height_;
	}

	void setResolution(float resolution)
	{
		resolution_ = resolution;
	}

	float getResolution()
	{
		return resolution_;
	}

	void setOriginX(double origin_x)
	{
		origin_x_ = origin_x;
	}

	double getOriginX()
	{
		return origin_x_;
	}

	void setOriginY(double origin_y)
	{
		origin_y_ = origin_y;
	}

	double getOriginY()
	{
		return origin_y_;
	}

	void setData(std::vector<int8_t> data)
	{
		map_data_ = data;
	}

	void getData(std::vector<int8_t>& map_data)
	{
		map_data = map_data_;
	}

private:
	uint32_t width_;
	uint32_t height_;
	float resolution_;
	double origin_x_;
	double origin_y_;
	std::vector<int8_t> map_data_;
};

extern SlamMap slam_map;


#endif //PP_SLAM_MAP_H
