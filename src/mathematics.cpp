#include <stdint.h>
#include <math.h>
#include <event_manager.h>
#include <beeper.h>

#include "mathematics.h"
#include "ros/ros.h"

double ranged_radian(double radian)
{
	while (radian > PI || radian <= -PI)
	{
		if (radian > PI) {
			radian -= PI*2;
		} else
		if (radian <= -PI) {
			radian += PI*2;
		}
	}
	return radian;
}

double ranged_degree(double degree)
{
	while (degree > 180 || degree <= -180)
	{
		if (degree > 180) {
			degree -= 360;
		} else
		if (degree <= -180) {
			degree += 360;
		}
	}
	return degree;
}
double degree_to_radian(double deg)
{
	return (deg * PI / 180);
}

double radian_to_degree(double rad)
{
	return (rad * 180 / PI);
}

float two_points_distance_double(float startx,float starty,float destx,float desty)
{

	double d,e;

	d = (double)destx - (double)startx;
	e = (double)desty - (double)starty;
	d *= d;
	e *= e;
	return sqrt(d + e);

}

void matrix_translate(double *x, double *y, double offset_x, double offset_y)
{
	*x += offset_x;
	*y += offset_y;
}

void matrix_rotate(double *x, double *y, double theta)
{
	double d, e;

	d = (*x) * cos(theta) - (*y) * sin(theta);
	e = (*x) * sin(theta) + (*y) * cos(theta);

	*x = d;
	*y = e;
}

//Line's angle, range is (-pi/2, pi/2]
//Mode other value: easy is_max_clean_state_, only calculate k = A/B;
//Mode 1: Precise Mode, A or B = 1.0 needed
double line_angle(LineABC l, uint8_t mode) {
	double tmp;
	if ( mode == 1 ) {
		if ( l.A == 1.0 ) {
			tmp = PI / 2 - atan(-l.B);
			if ( tmp > PI / 2 )
				tmp -= PI;
		} else if ( l.B == 1.0 ) {
			tmp = atan(-l.A);
		} else {
			if ( std::abs(l.B) < 0.000001 ) {
				tmp = PI / 2;
			} else {
				tmp = atan(- l.A/l.B);
			}
		}
	} else {
		if ( l.B != 0.0 ) {
			tmp = atan(-l.A / l.B);
		} else
			tmp = PI / 2;
	}
	return tmp;
}

void coordinate_transform(double *x, double *y, double theta, double offset_x, double offset_y)
{
	matrix_rotate(x, y, theta);
	matrix_translate(x, y, offset_x, offset_y);
}

bool unsigned_long_to_hex_string(unsigned long number, char *str, const int len)
{
	if (len < 3)
	{
		ROS_ERROR("%s %d: Input string length is less then 3.", __FUNCTION__, __LINE__);
		return false;
	}

	char base[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
	int i = 0, j = 0;
	int tmp_len = len - 2; // 2 for '0x'.
	char tmp_str[tmp_len]{};

	while (number != 0 && j < tmp_len - 1)
	{
		tmp_str[j] = base[number % 16];
		number /= 16;
		j++;
	}
//	ROS_INFO("tmp_str: %s", tmp_str);

	if (number != 0 && j == tmp_len - 1)
	{
		ROS_ERROR("%s %d: Input string length is too short to load the number.", __FUNCTION__, __LINE__);
		return false;
	}

	str[i++] = '0';
	str[i++] = 'x';
	j--;
	while (j >= 0)
	{
		str[i++] = tmp_str[j];
		j--;
	}

	str[i] = '\0';
//	ROS_INFO("string after convert: %s", str);

	return true;
}

// Functions for class PointSelector
PointSelector::PointSelector(bool is_left, double wall_distance)
{
	is_left_ = is_left;
//	narrow = is_left ? 0.177 : 0.187;
	const auto wall_dis_offset = 0.00;
	double w_l = wall_distance - wall_dis_offset;
	double w_r = wall_distance + 0.01 - wall_dis_offset;
	check_limit(w_l, 0.167, 0.187);
	check_limit(w_r, 0.177, 0.197);
//	narrow = is_left ?  w_l : w_r;
	narrow = is_left ? 0.167 : 0.182;
	if (wall_distance > 0.180) {
		narrow = narrow + 0.01;
//		beeper.debugBeep();
	}
//	ROS_WARN("narrow = %lf", narrow);
	narrow_minuend = is_left ? 0.03 : 0.03;

	x_min_forward = LIDAR_OFFSET_X;
	x_max_forward = is_left ? 0.3 : 0.3;
	auto y_start_forward = is_left ? 0.06 : -0.06;
	auto y_end_forward = is_left ? -ROBOT_RADIUS : ROBOT_RADIUS;
	y_min_forward = std::min(y_start_forward, y_end_forward);
	y_max_forward = std::max(y_start_forward, y_end_forward);

	auto x_side_start = 0.0;
	auto x_side_end = ROBOT_RADIUS;
	x_min_side = std::min(x_side_start, x_side_end);
	x_max_side = std::max(x_side_start, x_side_end);

	auto y_side_start = 0.0;
	auto y_side_end = is_left ? narrow + 0.01 : -narrow - 0.01;
	y_min_side = std::min(y_side_start, y_side_end);
	y_max_side = std::max(y_side_start, y_side_end);

	auto y_point1_start_corner = is_left ? 0.3 : -0.3;
	auto y_point1_end_corner = is_left ? -4.0 : 4.0;
	y_min_point1_corner = std::min(y_point1_start_corner, y_point1_end_corner);
	y_max_point1_corner = std::max(y_point1_start_corner, y_point1_end_corner);

	auto y_point1_start = 0.0;
	auto y_point1_end = is_left ? 4.0 : -4.0;
	y_min_point1 = std::min(y_point1_start, y_point1_end);
	y_max_point1 = std::max(y_point1_start, y_point1_end);

	auto y_target_start = is_left ? ROBOT_RADIUS : -ROBOT_RADIUS;
	auto y_target_end = is_left ? 0.4 : -0.4;
	y_min_target = std::min(y_target_start, y_target_end);
	y_max_target = std::max(y_target_start, y_target_end);

	corner_front_trig_lim = is_left ? 0.25 : 0.25;

}
bool PointSelector::LaserPointRange(const Vector2<double> &point, bool is_corner) const
{
/*		if (point.Distance({0, 0}) <= ROBOT_RADIUS) {
			return false;
		}*/
	if (is_corner)
		return (point.x > 0 && point.x < 4 && point.y > y_min_point1_corner && point.y < y_max_point1_corner);
	else
		return (point.x > 0 && point.x < 0.3 && point.y > y_min_point1 && point.y < y_max_point1);
}

bool PointSelector::TargetPointRange(const Vector2<double> &target)
{
	if (is_left_)
	{
		return /*(target.x > ROBOT_RADIUS && target.y < 0.4 && target.y > ROBOT_RADIUS) ||*/
				(target.x > CHASE_X && std::abs(target.y) < ROBOT_RADIUS) ||
				(target.y < -ROBOT_RADIUS);
	} else
	{
		return /*(target.x > ROBOT_RADIUS && target.y > -0.4 && target.y < -ROBOT_RADIUS) ||*/
				(target.x > CHASE_X && std::abs(target.y) < ROBOT_RADIUS) ||
				(target.y > ROBOT_RADIUS);
	}
}

bool PointSelector::inForwardRange(const Vector2<double> &point) const
{
	return point.x > x_min_forward && point.x < x_max_forward && point.y > y_min_forward && point.y < y_max_forward;
}

bool PointSelector::inSidedRange(const Vector2<double> &point) const
{
	return point.x > x_min_side && point.x < x_max_side && point.y > y_min_side && point.y < y_max_side;
}

Vector2<double> polarToCartesian(double polar, int i)
{
	Vector2<double> point{cos(degree_to_radian(i * 1.0 + 180.0)) * polar,
												sin(degree_to_radian(i * 1.0 + 180.0)) * polar };

	coordinate_transform(&point.x, &point.y, LIDAR_THETA, LIDAR_OFFSET_X, LIDAR_OFFSET_Y);
	return point;
}


Dir_t get_dir(const Cells::iterator& neighbor, const Cells::iterator& curr)
{
    get_dir(*neighbor ,*curr);
}
Dir_t get_dir(const Cell_t& neighbor, const Cell_t& curr)
{

	assert(neighbor.x != curr.x || neighbor.y != curr.y);

	if(neighbor.x != curr.x && neighbor.y != curr.y)
	{
        if(neighbor.x > curr.x && neighbor.y>curr.y)
			return MAP_PX_PY;
		else if(neighbor.x > curr.x && neighbor.y<curr.y)
			return MAP_PX_NY;
		else if(neighbor.x < curr.x && neighbor.y>curr.y)
			return MAP_NX_PY;
		else if(neighbor.x < curr.x && neighbor.y<curr.y)
			return MAP_NX_NY;
	}


	if(neighbor.y == curr.y)
        return neighbor.x  > curr.x ? MAP_POS_X : MAP_NEG_X;

	if (neighbor.x == curr.x)
		return neighbor.y  > curr.y ? MAP_POS_Y : MAP_NEG_Y;
}

float cellToCount(int16_t i) {
	return i * CELL_SIZE;
}

std::unique_ptr<Cells> points_to_cells(const Points& points)
{
	auto cells = make_unique<Cells>();
	for(auto&& point :points)
	{
		cells->emplace_back(point.toCell());
	}
	return cells;
};

std::unique_ptr<Points> cells_to_points(const Cells& path)
{
//	displayCellPath(*path);
	auto  point_path = make_unique<Points>();
	std::string debug_str;
	if(!path.empty()){
		for(auto&& it = path.begin(); it != path.end(); ++it) {
			Point_t target {cellToCount((*it).x),cellToCount((*it).y),0};
			if(it == path.end()-1)
			{
				if(point_path->empty())
					point_path->emplace_back(target);
				target.dir = point_path->back().dir;
				target.th = point_path->back().th;
			}else {
				auto it_next = it+1;
				if (it->x == it_next->x) {
					target.dir = it->y > it_next->y ? MAP_NEG_Y : MAP_POS_Y;
					target.th = isPos(target.dir) ? PI / 2 : -PI / 2;
				} else {
					target.dir = it->x > it_next->x ? MAP_NEG_X : MAP_POS_X;
					target.th = isPos(target.dir) ? 0 : PI;
				}
			}
			point_path->emplace_back(target);
			debug_str += "(" + std::to_string(it->x) + ", " + std::to_string(it->y) + ")";
		}
	}
	ROS_INFO("%s %d: it:%s, cell.back(%d,%d)",__FUNCTION__, __LINE__, debug_str.c_str(),
			 path.back().x, path.back().y/*, path->back().th*/);
	return point_path;
}

bool isAny(Dir_t dir)
{
	return dir == MAP_ANY;
}

bool isPos(Dir_t dir)
{
	return dir == MAP_POS_X || dir == MAP_POS_Y;
}

bool isXAxis(Dir_t dir)
{
	return dir == MAP_POS_X || dir == MAP_NEG_X;
}
void displayCellPath(const Cells &path)
{
	std::string     msg = __FUNCTION__;

	msg += " " + std::to_string(__LINE__) + ": Path size(" + std::to_string(path.size()) + "):";
	for (auto it = path.begin(); it != path.end(); ++it) {
		msg += "{" + std::to_string(it->x) + ", " + std::to_string(it->y) + "},";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

void displayTargetList(const Cells &target_list)
{
	std::string     msg = __FUNCTION__;
	msg += " " + std::to_string(__LINE__) + ": targets = {" + std::to_string(target_list.size()) + "}:";
	for (auto it = target_list.begin(); it != target_list.end(); ++it) {
		msg += "{" + std::to_string(it->x) + ", " + std::to_string(it->y) + ", " + "},";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

void displayPointPath(const Points &point_path)
{
	std::string     msg = __FUNCTION__;
	msg += " " + std::to_string(__LINE__) + ": Points(" + std::to_string(point_path.size()) + "):";
	for (auto it = point_path.begin(); it != point_path.end(); ++it) {
		msg += "(" + std::to_string((it->toCell().x)) + ", " + std::to_string(it->toCell().y) + ", " + std::to_string(
						static_cast<int>(radian_to_degree(it->th)))+ ", " + std::to_string(it->dir) + "),";
	}
	//msg += "\n";
	ROS_INFO("%s",msg.c_str());
}

bool is_opposite_dir(int l, int r)
{
	return (l == 0 && r==1)  || (l ==1 && r ==0) || (l ==2 && r ==3) || (l == 3 && r == 2);
}

void HomePointsManager::setRconPoint(const Point_t &point) {
	if (std::none_of(home_points_list_[0].begin(), home_points_list_[0].end(), [&](const Point_t &it) {
		return it.toCell() == point.toCell();
	}) && point.toCell() != Cell_t{})
	{
		home_points_list_[0].push_front(point);
		home_points_it_ = home_points_list_.begin();
		home_point_it_ = home_points_list_[0].begin();
		ROS_INFO("%s %d: Set home cell.%d,%d",__FUNCTION__, __LINE__, point.toCell().x, point.toCell().y);
	}
}

void HomePointsManager::popCurrRconPoint() {
	ROS_WARN("%s %d: 1 size(%d): %d,%d",__FUNCTION__, __LINE__, home_points_list_[0].size(), home_point_it_->toCell().x, home_point_it_->toCell().y);
	for(auto&& it :  home_points_list_[0])
	{
		ROS_INFO("%d,%d ", it.toCell().x,it.toCell().y);
	}
	home_points_list_[0].erase(home_point_it_/*++*/);
	if (home_point_it_ == home_points_list_[0].end()) {
		ROS_ERROR("%s %d:3 size(%d)",__FUNCTION__, __LINE__, home_points_list_[0].size());
		if(home_points_list_[0].empty())
		{
			home_points_it_++;//point to start point
			home_point_it_ = home_points_list_[1].begin();
		}else{
			home_point_it_ = home_points_list_[0].begin();
		}
	}
	ROS_WARN("%s %d: 2 size(%d): curr_point(%d,%d)",__FUNCTION__, __LINE__, home_points_list_[0].size(), home_point_it_->toCell().x, home_point_it_->toCell().y);
}

HomePointsManager::HomePointsManager() {
	home_points_list_[1].push_back(Point_t{0,0});
	home_point_it_ = home_points_list_[1].begin();
}

void HomePointsManager::for_each(const std::function<void(const Point_t &it)> &lambda_fun) {
	for (auto &&p_home_points_it : home_points_list_) {
		for (auto &&it = p_home_points_it.begin(); it != p_home_points_it.end(); ++it) {
			lambda_fun(*it);
		}
	}
}

void HomePointsManager::resetRconPoint() {
	if (!home_points_list_[0].empty()) {
		home_points_list_[0].clear();
	}
	home_point_it_ = home_points_list_[1].begin();
}

void HomePointsManager::setStartPointRad(double th) {
		home_points_list_[1].begin()->th = th;
}

double HomePointsManager::getStartPointRad() {
		return home_points_list_[1].begin()->th;
}

HomePoints_t::iterator HomePointsManager::getStartPoint() {
	return home_points_list_[1].begin();
}

bool HomePointsManager::isStartPoint() {
	return home_points_it_ != home_points_list_.begin();
}
