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

bool MovementFollowWallLidar::calcTmpTarget(Point32_t& tmp_target) {
	auto p_mt = (ActionFollowWall *) sp_mt_;

	auto scan = Lidar::getLidarScanDataOriginal();
	if (scan.header.seq == seq_) {
		if (p_mt->tmp_plan_path_.empty())
			return false;
		tmp_target = p_mt->tmp_plan_path_.front();
		if (std::abs(getPosition().X - tmp_target.X) < 30 && std::abs(getPosition().Y - tmp_target.Y) < 30) {
			p_mt->tmp_plan_path_.pop_front();
			if (p_mt->tmp_plan_path_.empty()) {
				return false;
			}
			tmp_target = p_mt->tmp_plan_path_.front();
		}
		return true;
	}
	seq_ = scan.header.seq;

	if (calcLidarPath(scan))
		return false;

	tmp_target = p_mt->tmp_plan_path_.front();
}

Vector2<double> MovementFollowWallLidar::polar_to_cartesian(double polar,int i)
{
	Vector2<double> point{cos((i * 1.0 + 180.0) * PI / 180.0) * polar,
					sin((i * 1.0 + 180.0) * PI / 180.0) * polar };

	coordinate_transform(&point.X, &point.Y, LIDAR_THETA, LIDAR_OFFSET_X, LIDAR_OFFSET_Y);
	return point;

}

bool MovementFollowWallLidar::check_corner(sensor_msgs::LaserScan scan, const Paras para) {
	int forward_wall_count = 0;
	int side_wall_count = 0;
	for (int i = 359; i > 0; i--) {
		if (scan.ranges[i] < 4) {
			auto point = polar_to_cartesian(scan.ranges[i], i);
			if (para.inForwardRange(point)) {
				forward_wall_count++;
//			ROS_INFO("point(%f,%f)",point.X,point.Y);
//				ROS_INFO("forward_wall_count(%d)",forward_wall_count);
			}
			if (para.inSidedRange(point)) {
				side_wall_count++;
//			ROS_INFO("point(%f,%f)",point.X,point.Y);
//				ROS_INFO("side_wall_count(%d)",side_wall_count);
			}
		}
	}
	return forward_wall_count > 10 && side_wall_count > 20;
}


Vector2<double> MovementFollowWallLidar::get_middle_point(const Vector2<double>& p1,const Vector2<double>& p2,const Paras& para) {
	auto p3 = (p1 + p2) / 2;
	Vector2<double> target{};

//	ROS_INFO("p1(%f,%f)", p1.X, p1.Y);
//	ROS_INFO("p2(%f,%f)", p2.X, p2.Y);
//	ROS_INFO("p3 (%f,%f)", p3.X, p3.Y);

//	auto x4 = para.narrow / (sqrt(1 + p1.SquaredDistance(p2))) + p3.X;
//	auto y4 = ((x4 - p3.X) * (p1.X - p2.X) / (p2.Y - p1.Y)) + p3.Y;
	auto dx = para.narrow / (sqrt(1 + ((p1.X - p2.X) / (p2.Y - p1.Y)) * ((p1.X - p2.X) / (p2.Y - p1.Y))));
	auto x4 = dx + p3.X;

	auto dy = (x4 - p3.X) * (p1.X - p2.X) / (p2.Y - p1.Y);
	auto y4 = dy + p3.Y;

//	ROS_INFO("x4,y4(%f,%f)", x4, y4);

	if (((p1.X - x4) * (p2.Y - y4) - (p1.Y - y4) * (p2.X - x4)) < 0) {
		target = {x4,y4};
//		ROS_INFO_FL();
//		ROS_INFO("target(%f,%f)", target.X, target.Y);
	}
	else {
		x4 =  -dx + p3.X;
		y4 = (x4 - p3.X) * (p1.X - p2.X) / (p2.Y - p1.Y) + p3.Y;
		target = {x4,y4};
//		ROS_INFO_FL();
//		ROS_INFO("target(%f,%f)", target.X, target.Y);
	}
	return target;
}

bool MovementFollowWallLidar::check_is_valid(const Vector2<double>& point, Paras& para, sensor_msgs::LaserScan& scan) {
	for (int i = 359; i >= 0; i--) {
		auto tmp_point = polar_to_cartesian(scan.ranges[i], i);
		auto distance = point.Distance(tmp_point);
		//ROS_INFO("distance =  %lf", distance);
		if (distance < para.narrow - 0.03) {
			return false;
		}
	}
	return true;
}

bool MovementFollowWallLidar::calcLidarPath(sensor_msgs::LaserScan scan) {
	std::deque<Vector2<double>> points{};
	Paras para{is_left_};

	auto is_corner = check_corner(scan, para);
	ROS_WARN("is_corner = %d", is_corner);
	for (int i = 359; i >= 0; i--) {
		//ROS_INFO("i = %d", i);
		if (scan.ranges[i] < 4 && scan.ranges[i - 1] < 4) {
			auto point1 = polar_to_cartesian(scan.ranges[i], i);

			if (!para.inPoint1Range(point1, is_corner))
				continue;

			auto point2 = polar_to_cartesian(scan.ranges[i - 1], i - 1);

			if (point2.Distance(point1) > 0.05) {
				//ROS_INFO("two points distance is too large");
				continue;
			}
			auto target = get_middle_point(point1, point2, para);

			if (!para.inTargetRange(target))
				continue;

			if (target.Distance({0, 0}) > 0.4)
				continue;

			if (!check_is_valid(target, para, scan))
				continue;

//			ROS_INFO("points(%d):target(%lf,%lf),dis(%f)", points.size(), target.X, target.Y, target.Distance({CHASE_X, 0}));
			points.push_back(target);
		}
	}

	if (points.empty()) {
		return false;
	}
	if (!is_left_) {
		std::reverse(points.begin(), points.end());//for the right wall follow
	}
	auto min = std::min_element(points.rbegin(), points.rend(), [](Vector2<double>& a, Vector2<double>& b) {
//		ROS_INFO("dis(%f,%f)", a.Distance({CHASE_X, 0}), b.Distance({CHASE_X, 0}));
		return a.Distance({CHASE_X, 0}) < b.Distance({CHASE_X, 0});
	});
//	ROS_INFO("min(%f,%f)",min->X, min->Y);

	auto size = points.size();
	std::copy(points.rbegin(), min+1, std::front_inserter(points));
	points.resize(size);
//	for (const auto &target :points)
//	{
//		ROS_WARN("points(%d):target(%lf,%lf),dis(%f)", points.size(), target.X, target.Y, target.Distance({CHASE_X, 0}));
//	}
	robot::instance()->pubPointMarkers(&points, "base_link");

	auto p_mt = (ActionFollowWall*)sp_mt_;
	p_mt->tmp_plan_path_.clear();
	for (const auto& iter : points) {
		p_mt->tmp_plan_path_.push_back(getRelative(getPosition(), int(iter.Y * 1000), int(iter.X * 1000), true));
	}

	return true;
}

bool MovementFollowWallLidar::isFinish() {
//	return false;
	auto p_mt = (ActionFollowWall*)(sp_mt_);
	return p_mt->tmp_plan_path_.empty() || p_mt->shouldMoveBack() || p_mt->shouldTurn();
}

