//
// Created by lsy563193 on 12/19/17.
//
#include "pp.h"
#include "arch.hpp"

#define WF_SPEED						((int32_t) 20)
#define WF_SCAN_TYPE						(2)
Points g_wf_path{};

const double CHASE_X = 0.107;

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

MovementFollowWallLidar::MovementFollowWallLidar(bool is_left) : MovementFollowWallInfrared(is_left)
{
	tmp_target_ = calcTmpTarget();
}

bool MovementFollowWallLidar::getLidarWfTarget2(std::vector<Vector2<double>> &points) {
	static uint32_t seq = 0;
	bool is_left = true;
	Paras para{is_left};
	auto scan = Lidar::getLidarScanDataOriginal();

	if (scan.header.seq == seq) {
		return false;
	}
	seq = scan.header.seq;
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
	if (!is_left) {
		std::reverse(points.begin(), points.end());//for the right wall follow
	}
	std::sort(points.begin(), points.end(), [](Vector2<double> a, Vector2<double> b) {
		return a.Distance({CHASE_X, 0}) < b.Distance({CHASE_X, 0});
	});
	robot::instance()->pubPointMarkers(&points, scan.header.frame_id);
	return true;
}

Point32_t MovementFollowWallLidar::calcTmpTarget()
{
		std::vector<Vector2<double>> g_wf_target_point{};
		g_wf_path.clear();
		getLidarWfTarget2(g_wf_target_point);
		for (auto iter = g_wf_target_point.begin(); iter != g_wf_target_point.end(); ++iter) {
			Point32_t p_target = GridMap::getRelative(GridMap::getCurrPoint(),int(iter->Y * 1000),int(iter->X * 1000),true);
			g_wf_path.push_back(p_target);
		}
	auto curr = nav_map.getCurrPoint();
	extern boost::mutex scan2_mutex_;
	PPTargetType path;
	scan2_mutex_.lock();
	if (is_sp_turn)
	{
	}
	else
	{
		if (g_wf_path.empty())
		{
			is_no_target = true;
		}
		else
		{
			if (std::abs(curr.X - tmp_target_.X) < 30 && std::abs(curr.Y - tmp_target_.Y) < 30) {
				if (g_wf_path.size() > 0) {
					g_wf_path.pop_front();
					tmp_target_ = g_wf_path.front();
				}
				ROS_ERROR("reach! g_wf_path.points.pop_front(), size = %d", g_wf_path.size());
				is_no_target = g_wf_path.empty();
			}

			is_no_target = false;
			if (tmp_target_.X != g_wf_path.front().X && tmp_target_.Y != g_wf_path.front().Y) {
				tmp_target_ = g_wf_path.front();
				ROS_INFO("tmp_target_ = g_wf_path.points.front()");
			}
		}
	}
	scan2_mutex_.unlock();
	return tmp_target_;
}

void MovementFollowWallLidar::adjustSpeed(int32_t &left_speed, int32_t &right_speed) {
	wheel.setDirectionForward();
	if (is_no_target) {
		if (is_left_) {
			left_speed = 8;
			right_speed = 31;
		}
		else {
			left_speed = 31;
			right_speed = 8;
		}
		ROS_WARN("not target:left_speed(%d),right_speed(%d)", left_speed, right_speed);
	}
	else {
		ROS_ERROR("find target:left_speed(%d),right_speed(%d)", left_speed, right_speed);
		auto curr = GridMap::getCurrPoint();
		auto angle_diff = ranged_angle(
						course_to_dest(curr.X, curr.Y, tmp_target_.X, tmp_target_.Y) - GridMap::getCurrPoint().TH);

		if (integration_cycle_++ > 10) {
			integration_cycle_ = 0;
			integrated_ += angle_diff;
			check_limit(integrated_, -150, 150);
		}

		int kp;
		auto tmp_cond = is_left_ ? (angle_diff < -600 || is_sp_turn) : (angle_diff > 600 || is_sp_turn);
		auto tmp_cond_1 = is_left_ ? (angle_diff > 450) : (angle_diff < -450);
		if (tmp_cond) {
			beeper.play_for_command(true);
			if (!is_sp_turn) {
				tmp_target_ = calcTmpTarget();
				ROS_INFO("fresh tmp_target_!");
			}
			auto tmp_angle_diff = ranged_angle(
							course_to_dest(curr.X, curr.Y, tmp_target_.X, tmp_target_.Y) - GridMap::getCurrPoint().TH);
			ROS_INFO("tmp_angle_diff = %d angle_diff = %d", tmp_angle_diff, angle_diff);
			auto tmp_cond_2 = is_left_ ? (tmp_angle_diff > -10) : (tmp_angle_diff < 10);
			is_sp_turn = !tmp_cond_2;
			int speed_lim = 0;
			if (base_speed_ > speed_lim) {
				base_speed_--;
			}

			if (is_left_) {
				left_speed = 10;
				right_speed = 0 - left_speed;
			}
			else {
				right_speed = 10;
				left_speed = 0 - right_speed;
			}
			base_speed_ = (left_speed + right_speed) / 2;
		}
		else if (tmp_cond_1) {
			if (is_left_) {
				left_speed = 8;
				right_speed = 31;
			}
			else {
				left_speed = 31;
				right_speed = 8;
			}
			base_speed_ = (left_speed + right_speed) / 2;
		}
		else {
			//kp = 20;
			kp = 20;//20
			if (base_speed_ < WF_SPEED) {
				base_speed_++;
			}
			if (is_left_) {
				left_speed = base_speed_ - angle_diff / kp -
										 integrated_ / 150; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
				right_speed = base_speed_ + angle_diff / kp +
											integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;
			}
			else {
				left_speed = base_speed_ - angle_diff / kp -
										 integrated_ / 150; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
				right_speed = base_speed_ + angle_diff / kp +
											integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;
			}
			base_speed_ = (left_speed + right_speed) / 2;
		}
	}
}

