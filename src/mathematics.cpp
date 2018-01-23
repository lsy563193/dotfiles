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
