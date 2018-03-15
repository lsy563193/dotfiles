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
//Mode other value: easy mode_, only calculate k = A/B;
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
//		beeper.beepForCommand(VALID);
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
