//
// Created by lsy563193 on 12/19/17.
//
#include "pp.h"
#include "arch.hpp"

#define WF_SCAN_TYPE						(2)

const double CHASE_X = 0.107;

MovementFollowWallLidar::MovementFollowWallLidar(bool is_left)
				: IFollowWall(is_left)
{
	min_speed_ = FALL_WALL_MIN_SPEED;
	max_speed_ = FALL_WALL_MAX_SPEED;
	base_speed_ = min_speed_;
	tick_limit_ = 0;

//	path_thread_ = new boost::thread(boost::bind(&MovementFollowWallLidar::calcTmpTarget));
//	path_thread_->detach();
}

Point32_t MovementFollowWallLidar::calcTmpTarget() {
	if(calcLidarPath())
	{
		if (!tmp_plan_path_.empty()) {
			auto curr = nav_map.getCurrPoint();
			if (std::abs(curr.X - tmp_plan_path_.front().X) < 30 && std::abs(curr.Y - tmp_plan_path_.front().Y) < 30) {
				tmp_plan_path_.pop_front();
			}
		}
	}
	return tmp_plan_path_.front();
}

Vector2<double> MovementFollowWallLidar::polar_to_cartesian(double polar,int i)
{
	Vector2<double> point{cos((i * 1.0 + 180.0) * PI / 180.0) * polar,
					sin((i * 1.0 + 180.0) * PI / 180.0) * polar };

	coordinate_transform(&point.X, &point.Y, LIDAR_THETA, LIDAR_OFFSET_X, LIDAR_OFFSET_Y);
	return point;

}

bool MovementFollowWallLidar::check_obstacle(sensor_msgs::LaserScan scan, const Paras para) {
	int forward_wall_count = 0;
	int side_wall_count = 0;
	for (int i = 359; i > 0; i--) {
		if (scan.ranges[i] < 4) {
			auto point = polar_to_cartesian(scan.ranges[i], i);
			if (para.inForwardRange(point)) {
				forward_wall_count++;
			}
			if (para.inSidedRange(point)) {
				side_wall_count++;
			}
		}
	}
	return forward_wall_count > 10 && side_wall_count > 20;
}


Vector2<double> MovementFollowWallLidar::get_middle_point(Vector2<double> p1, Vector2<double> p2,Paras para) {
	auto cart3 = (p1 + p2) / 2;
	Vector2<double> p;

	auto x_4 = para.narrow / (sqrt(1 + p1.SquaredDistance(p2))) + cart3.X;
	auto y_4 = ((x_4 - cart3.X) * (p1.X - p2.X) / (p2.Y - p1.Y)) + cart3.Y;

	if (((p1.X - x_4) * (p2.Y - y_4) - (p1.Y - y_4) * (p2.X - x_4)) < 0) {
		p.X = x_4;
		p.Y = y_4;
	}
	else {
		x_4 = 0 - para.narrow / (sqrt(1 + p1.SquaredDistance(p2))) + cart3.X;
		y_4 = ((x_4 - cart3.X) * (p1.X - p2.X) / (p2.Y - p1.Y)) + cart3.Y;

		p.X = x_4;
		p.Y = y_4;
	}
	return p;
}

bool MovementFollowWallLidar::check_is_valid(Vector2<double> point, Paras para, sensor_msgs::LaserScan scan) {
	for (int j = 359; j > 0; j--) {
		auto cart_j = polar_to_cartesian(scan.ranges[j], j);
		auto distance = point.Distance(cart_j);
		//ROS_INFO("distance =  %lf", distance);
		if (distance < para.narrow - 0.03) {
			return false;
		}
	}
	return true;
}

bool MovementFollowWallLidar::calcLidarPath() {
	std::vector<Vector2<double>> points{};
	Paras para{is_left_};
	auto scan = Lidar::getLidarScanDataOriginal();

	if (scan.header.seq == seq_) {
		return false;
	}
	seq_ = scan.header.seq;
	points.clear();

	auto is_obstacle = check_obstacle(scan, para);
	ROS_WARN("is_obstacle = %d", is_obstacle);
	for (int i = 359; i >= 0; i--) {
		//ROS_INFO("i = %d", i);
		if (scan.ranges[i] < 4 && scan.ranges[i - 1] < 4) {
			auto point1 = polar_to_cartesian(scan.ranges[i], i);

			if (!para.inRange(point1))
				continue;

			auto point2 = polar_to_cartesian(scan.ranges[i - 1], i - 1);

			if (point2.Distance(point1) > 0.05) {
				//ROS_INFO("two points distance is too large");
				continue;
			}
			auto target = get_middle_point(point1, point2, para);

			if(para.inTargetRange(target))
				continue;

			if(target.Distance({0,0})<=0.4)
				continue;

			if (!check_is_valid(target, para, scan))
				continue;

			points.push_back(target);
		}
	}

	if (points.empty()) {
		return false;
	}
	if (!is_left_) {
		std::reverse(points.begin(), points.end());//for the right wall follow
	}
	std::sort(points.begin(), points.end(), [](Vector2<double> a, Vector2<double> b) {
		return a.Distance({CHASE_X, 0}) < b.Distance({CHASE_X, 0});
	});
	robot::instance()->pubPointMarkers(&points, scan.header.frame_id);

	tmp_plan_path_.clear();
	for (const auto& iter : points) {
		tmp_plan_path_.push_back(GridMap::getRelative(GridMap::getCurrPoint(), int(iter.Y * 1000), int(iter.X * 1000), true));
	}

	return true;
}

bool MovementFollowWallLidar::isFinish() {
	return false;
}
