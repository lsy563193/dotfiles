#ifndef __MYMATH_H
#define __MYMATH_H

#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <deque>
#include "config.h"

#define PI  3.141592653589793

#ifndef M_PI

#define M_PI	3.141592653589793

#endif

/*
typedef struct Pose16_t_{
	int16_t X;
	int16_t Y;
	int16_t	th;
	friend bool operator==(const Pose16_t_ left, const Pose16_t_ right)
	{
		return left.X == right.X && left.Y == right.Y;
	}
	friend bool operator!=(const Pose16_t_ left, const Pose16_t_ right)
	{
		return !(left == right);
	}
} Pose16_t;*/

int16_t ranged_angle(int16_t angle);
double deg_to_rad(double deg, int8_t scale = 1);
double rad_2_deg(double rad, int8_t scale);

  /**
   * Represents a 2-dimensional vector (x, y)
   */
 template<typename T>
 class Vector2
{
  public:
    /**
     * Vector at the origin
     */
    Vector2()
    {
      X = 0;
      Y = 0;
    }

    /**
     * Vector at the given location
     * @param x x
     * @param y y
     */
    Vector2(T x, T y)
    {
      X = x;
      Y = y;
    }

  public:
    /**
     * Gets the x-coordinate of this vector
     * @return the x-coordinate of the vector
     */
    inline const T& GetX() const
    {
      return X;
    }

    /**
     * Sets the x-coordinate of this vector
     * @param x the x-coordinate of the vector
     */
    inline void SetX(const T& x)
    {
      X = x;
    }

    /**
     * Gets the y-coordinate of this vector
     * @return the y-coordinate of the vector
     */
    inline const T& GetY() const
    {
      return Y;
    }

    /**
     * Sets the y-coordinate of this vector
     * @param y the y-coordinate of the vector
     */
    inline void SetY(const T& y)
    {
      Y = y;
    }

    /**
     * Floor point operator
     * @param rOther vector
     */
    inline void MakeFloor(const Vector2& rOther)
    {
      if (rOther.X < X) X = rOther.X;
      if (rOther.Y < Y) Y = rOther.Y;
    }

    /**
     * Ceiling point operator
     * @param rOther vector
     */
    inline void MakeCeil(const Vector2& rOther)
    {
      if (rOther.X > X) X = rOther.X;

      if (rOther.Y > Y) Y = rOther.Y;
    }

    /**
     * Returns the square of the length of the vector
     * @return square of the length of the vector
     */
    inline T SquaredLength() const
    {
      return pow(X,2) + pow(Y,2);
    }

    /**
     * Returns the length of the vector
     * @return length of the vector
     */
    inline T Length() const
    {
      return sqrt(SquaredLength());
    }

    /**
     * Returns the square of the distance to the given vector
     * @param rOther vector
     * @returns square of the distance to the given vector
     */
    inline T SquaredDistance(const Vector2& rOther) const
    {
      return (*this - rOther).SquaredLength();
    }

    /**
     * Gets the distance to the given vector
     * @param rOther vector
     * @return distance to given vector
     */
    inline T Distance(const Vector2& rOther) const
    {
      return sqrt(SquaredDistance(rOther));
    }
/*

    */
/**
     * Returns a string representation of this vector
     * @return string representation of this vector
     */

/*
    inline const String ToString() const
    {
      String valueString;
      valueString.Append(StringHelper::ToString(GetX()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetY()));
      return valueString;
    }
*/

  public:
    /**
     * In-place vector addition
     */
    inline void operator+=(const Vector2& rOther)
    {
      X += rOther.X;
      Y += rOther.Y;
    }

    /**
     * In-place vector subtraction
     */
    inline void operator-=(const Vector2& rOther)
    {
      X -= rOther.X;
      Y -= rOther.Y;
    }

    /**
     * Vector addition
     */
    inline const Vector2 operator+(const Vector2& rOther) const
    {
      return Vector2(X + rOther.X, Y + rOther.Y);
    }

    /**
     * Vector subtraction
     */
    inline const Vector2 operator-(const Vector2& rOther) const
    {
      return Vector2(X - rOther.X, Y - rOther.Y);
    }

    /**
     * In-place scalar division
     */
    inline void operator/=(T scalar)
    {
      X /= scalar;
      Y /= scalar;
    }

    /**
     * Divides a vector by the scalar
     */
    inline const Vector2 operator/(T scalar) const
    {
      return Vector2(X / scalar, Y / scalar);
    }

    /**
     * Vector dot-product
     */
    inline int16_t operator*(const Vector2& rOther) const
    {
      return X * rOther.X + Y * rOther.Y;
    }

    /**
     * Scales the vector by the given scalar
     */
    inline const Vector2 operator*(T scalar) const
    {
      return Vector2(X * scalar, Y * scalar);
    }

    /**
     * Subtract the vector by the given scalar
     */
    inline const Vector2 operator-(T scalar) const
    {
      return Vector2(X - scalar, Y - scalar);
    }

    /**
     * In-place scalar multiplication
     */
    inline void operator*=(T scalar)
    {
      X *= scalar;
      Y *= scalar;
    }

    /**
     * Equality operator
     */
    inline bool operator==(const Vector2& rOther) const
    {
      return (X == rOther.X && Y == rOther.Y);
    }

    /**
     * Inequality operator
     */
    inline bool operator!=(const Vector2& rOther) const
    {
      return (X != rOther.X || Y != rOther.Y);
    }

    /**
     * Less than operator
     * @param rOther vector
     * @return true if left vector is 'less' than right vector by comparing corresponding x coordinates and then
     * corresponding y coordinates
     */
    inline bool operator<(const Vector2& rOther) const
    {
      if (X < rOther.X)
      {
        return true;
      }
      else if (X > rOther.X)
      {
        return false;
      }
      else
      {
        return (Y < rOther.Y);
      }
    }

    inline bool operator>(const Vector2& rOther) const
    {
        Vector2 this_{X,Y};
        return !(this_ < rOther || this_ == rOther);
    }
    /**
     * Write vector onto output stream
     */
//    friend std::ostream& operator<<(std::ostream& rStream, const Vector2& rVector)
//    {
//      rStream << rVector.ToString().ToCString();
//      return rStream;
//    }

  public:
    T X;
    T Y;
//    int16_t	th;
}; // class Vector2<T>

  /*
   * Type declaration of int16_t Vector2 as Cell_t
   */
typedef Vector2<int16_t> Cell_t;
typedef std::deque<Cell_t> Cells;

class Point32_t:public Vector2<int32_t> {
public:
  Point32_t() {
    X = 0;
    Y = 0;
    th = 0;
  }

  Point32_t(int32_t x, int32_t y, int16_t th) {
    X = x;
    Y = y;
    th = th;
  }

  Point32_t getRelative(int16_t dx, int16_t dy) const {
		Point32_t point;
		double relative_sin, relative_cos;
		if (th != 3600) {
			if (th == 0) {
				relative_sin = 0;
				relative_cos = 1;
			}
			else if (th == 900) {
				relative_sin = 1;
				relative_cos = 0;
			}
			else if (th == 1800) {
				relative_sin = 0;
				relative_cos = -1;
			}
			else if (th == -900) {
				relative_sin = -1;
				relative_cos = 0;
			}
			else {
				relative_sin = sin(deg_to_rad(th, 10));
				relative_cos = cos(deg_to_rad(th, 10));
			}
		}
		point.X = X + (int32_t) (
						(((double) dx * relative_cos * CELL_COUNT_MUL) - ((double) dy * relative_sin * CELL_COUNT_MUL)) /
						CELL_SIZE);
		point.Y = Y + (int32_t) (
						(((double) dx * relative_sin * CELL_COUNT_MUL) + ((double) dy * relative_cos * CELL_COUNT_MUL)) /
						CELL_SIZE);
		point.th = th;
		return point;
	}

  Cell_t toCell() const {
    return {countToCell(X), countToCell(Y)};
  }

  int16_t th{};
private:
  int16_t countToCell(int32_t count) const {
    if (count < -CELL_COUNT_MUL_1_2) {
      return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL - 1;
    }
    else {
      return (count + CELL_COUNT_MUL_1_2) / CELL_COUNT_MUL;
    }
  }
};

typedef struct
{
  double A;
  double B;
  double C;
  double len;
  double x1;//point 1
  double y1;
  double x2;//point 2
  double y2;
  double K; //gradient in degree
  double dist_2_this_line(Vector2<double> p){
	  return fabs(A*p.X+B*p.Y+C)/sqrt(A*A+B*B);
  }
} LineABC;

// line: y=Kx+B
typedef struct LineKB{
	float K;
	float B;
	float len;
	float x1;
	float y1;
	float x2;
	float y2;
	float get_x_by_y(const float y){
		return (float)(y-B)/K;
	}
	float get_y_by_x(const float x){
		return (float)(K*x)+B;
	}
} LineKB;

/*
typedef struct{
	int32_t X;
	int32_t Y;
  int16_t th;
} Point32_t;
*/

typedef struct{
	int32_t X;
	int32_t Y;
  int16_t TH;
} PointTh;


//typedef Vector2<int32_t> Point32_t;

uint16_t course_to_dest(const Point32_t& start, const Point32_t& dest);
uint32_t two_points_distance(int32_t startx, int32_t starty, int32_t destx, int32_t desty);
float two_points_distance_double(float startx,float starty,float destx,float desty);
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
void coordinate_transform(double *x, double *y, double theta, double offset_x, double offset_y);

int32_t abs_minus(int32_t A, int32_t B);

#endif
