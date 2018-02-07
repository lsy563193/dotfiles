#include <stdint.h>
#include <math.h>

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
