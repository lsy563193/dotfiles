#ifndef __MYMATH_H
#define __MYMATH_H

#include <stdint.h>

#include "mathematics.h"
#include "stdlib.h"

#define PI  3.141592653589793

#ifndef M_PI

#define M_PI	3.141592653589793

#endif

typedef struct Cell_t_{
	int16_t X;
	int16_t Y;
	friend bool operator==(const Cell_t_ left, const Cell_t_ right)
	{
		return left.X == right.X && left.Y == right.Y;
	}
	friend bool operator!=(const Cell_t_ left, const Cell_t_ right)
	{
		return !(left == right);
	}
} Cell_t;

typedef struct{
  double A;
  double B;
  double C;
} LineABC;

typedef struct{
	int32_t X;
	int32_t Y;
} Point32_t;

typedef struct{
	double x;
	double y;
} Double_Point;

typedef struct Pose16_t_{
	int16_t X;
	int16_t Y;
	int16_t	TH;
	friend bool operator==(const Pose16_t_ left, const Pose16_t_ right)
	{
		return left.X == right.X && left.Y == right.Y;
	}
	friend bool operator!=(const Pose16_t_ left, const Pose16_t_ right)
	{
		return !(left == right);
	}
} Pose16_t;

double absolute(double d);
double deg_to_rad(double deg, int8_t scale);
double rad_2_deg(double rad, int8_t scale);
uint16_t course_to_dest(int32_t startx, int32_t starty, int32_t destx, int32_t desty);
uint32_t two_points_distance(int32_t startx, int32_t starty, int32_t destx, int32_t desty);
int32_t two_points_distance_at_direction(int32_t startx, int32_t starty, int32_t destx, int32_t desty, int16_t theta);
int16_t distance2line(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t px, int32_t py);
uint16_t angle_delta(uint16_t a, uint16_t b);
int32_t limit(int32_t i, int32_t lower_limit, int32_t upper_limit);
void matrix_translate(double *x, double *y, double offset_x, double offset_y);
void matrix_rotate(double *x, double *y, double theta);
Cell_t cal_inters_point(Cell_t l1StartPnt, Cell_t l1EndPnt,
												Cell_t l2StartPnt, Cell_t l2EndPnt);

double rad_delta_angle_vector(double a, double b);
int16_t degree_delta_angle_vector(uint16_t a, uint16_t b) ;
double rad_delta_angle_min(double a, double b);
int16_t degreeDeltaAngleMin( uint16_t a, uint16_t b );

double arctan( double deltay, double deltax );
double two_lines_angle(LineABC la, LineABC lb);
double line_angle(LineABC l, uint8_t mode);
uint8_t is_same_point_and_angle(Point32_t pnt1, uint16_t angle1, Point32_t pnt2, uint16_t angle2,
																uint32_t pntThres, uint16_t angleThres);
#endif
