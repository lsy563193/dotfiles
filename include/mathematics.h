#ifndef __MYMATH_H
#define __MYMATH_H

#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <deque>
#include <ros/ros.h>
#include "config.h"
//#include "map.h"

#define PI M_PI

/*
typedef struct Pose16_t_{
	int16_t x;
	int16_t y;
	int16_t	th;
	friend bool operator==(const Pose16_t_ left, const Pose16_t_ right)
	{
		return left.x == right.x && left.y == right.y;
	}
	friend bool operator!=(const Pose16_t_ left, const Pose16_t_ right)
	{
		return !(left == right);
	}
} Pose16_t;*/

double ranged_radian(double radian);
double ranged_degree(double degree);
double degree_to_radian(double deg);
double radian_to_degree(double rad);

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
      x = 0;
      y = 0;
    }

    /**
     * Vector at the given location
     * @param x x
     * @param y y
     */
    Vector2(T _x, T _y)
    {
      x = _x;
      y = _y;
    }

  public:
    /**
     * Gets the x-coordinate of this vector
     * @return the x-coordinate of the vector
     */
    inline const T& GetX() const
    {
      return x;
    }

    /**
     * Sets the x-coordinate of this vector
     * @param x the x-coordinate of the vector
     */
    inline void SetX(const T& _x)
    {
      x = _x;
    }

    /**
     * Gets the y-coordinate of this vector
     * @return the y-coordinate of the vector
     */
    inline const T& GetY() const
    {
      return y;
    }

    /**
     * Sets the y-coordinate of this vector
     * @param y the y-coordinate of the vector
     */
    inline void SetY(const T& _y)
    {
      y = _y;
    }

    /**
     * Floor point operator
     * @param rOther vector
     */
    inline void MakeFloor(const Vector2& rOther)
    {
      if (rOther.x < x) x = rOther.x;
      if (rOther.y < y) y = rOther.y;
    }

    /**
     * Ceiling point operator
     * @param rOther vector
     */
    inline void MakeCeil(const Vector2& rOther)
    {
      if (rOther.x > x) x = rOther.x;

      if (rOther.y > y) y = rOther.y;
    }

    /**
     * Returns the square of the length of the vector
     * @return square of the length of the vector
     */
    inline T SquaredLength() const
    {
      return pow(x,2) + pow(y,2);
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
      x += rOther.x;
      y += rOther.y;
    }

    /**
     * In-place vector subtraction
     */
    inline void operator-=(const Vector2& rOther)
    {
      x -= rOther.x;
      y -= rOther.y;
    }

    /**
     * Vector addition
     */
    inline const Vector2 operator+(const Vector2& rOther) const
    {
      return Vector2(x + rOther.x, y + rOther.y);
    }

    /**
     * Vector subtraction
     */
    inline const Vector2 operator-(const Vector2& rOther) const
    {
      return Vector2(x - rOther.x, y - rOther.y);
    }

    /**
     * In-place scalar division
     */
    inline void operator/=(T scalar)
    {
      x /= scalar;
      y /= scalar;
    }

    /**
     * Divides a vector by the scalar
     */
    inline const Vector2 operator/(T scalar) const
    {
      return Vector2(x / scalar, y / scalar);
    }

    /**
     * Vector dot-product
     */
    inline int16_t operator*(const Vector2& rOther) const
    {
      return x * rOther.x + y * rOther.y;
    }

    /**
     * Scales the vector by the given scalar
     */
    inline const Vector2 operator*(T scalar) const
    {
      return Vector2(x * scalar, y * scalar);
    }

    /**
     * Subtract the vector by the given scalar
     */
    inline const Vector2 operator-(T scalar) const
    {
      return Vector2(x - scalar, y - scalar);
    }

    /**
     * In-place scalar multiplication
     */
    inline void operator*=(T scalar)
    {
      x *= scalar;
      y *= scalar;
    }

    /**
     * Equality operator
     */
    inline bool operator==(const Vector2& rOther) const
    {
      return (x == rOther.x && y == rOther.y);
    }

    /**
     * Inequality operator
     */
    inline bool operator!=(const Vector2& rOther) const
    {
      return (x != rOther.x || y != rOther.y);
    }

    /**
     * Less than operator
     * @param rOther vector
     * @return true if left vector is 'less' than right vector by comparing corresponding x coordinates and then
     * corresponding y coordinates
     */
    inline bool operator<(const Vector2& rOther) const
    {
      if (x < rOther.x)
      {
        return true;
      }
      else if (x > rOther.x)
      {
        return false;
      }
      else
      {
        return (y < rOther.y);
      }
    }

    inline bool operator>(const Vector2& rOther) const
    {
        Vector2 this_{x,y};
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
    T x;
    T y;
//    int16_t	th;
}; // class Vector2<T>

  /*
   * Type declaration of int16_t Vector2 as Cell_t
   */
typedef Vector2<int16_t> Cell_t;
typedef std::deque<Cell_t> Cells;

typedef enum {
	MAP_POS_X = 0,
//  const double MAP_NS_PY = PI*3/4;
	MAP_NEG_X,
//  const double MAP_PX_PY = PI/4;
	MAP_POS_Y,
//  const double MAP_NX_NY =-PI*3/4;
	MAP_NEG_Y,
//MAP_PX_NY =-PI/4;
	MAP_ANY,
} Dir_t;

class Point_t:public Vector2<float> {
public:
  Point_t() {
    x = 0;
    y = 0;
    th = 0;
  }
	Point_t(float _x, float _y) {
    x = _x;
    y = _y;
  }
  Point_t(float _x, float _y, double _th) {
    x = _x;
    y = _y;
    th = _th;
  }

	Point_t addRadian(double diff) const {
		return {this->x, this->y, ranged_radian(this->th + diff)};
	}
  Point_t getRelative(float dx, float dy) const {
		Point_t point{static_cast<float>(x + dx * cos(th) - dy * sin(th)), static_cast<float>(y + dx * sin(th) + dy * cos(th)), th};
//		return *this + point;
		return point;
	}

//	Point_t getRelative(float dx, float dy) const {
//		auto Cell = this->toCell();
//		Point_t point{cellToCount(Cell.x),cellToCount(Cell.y),th};
//		return point.getRelative(dx,dy);
//	}

	bool isNearTo(Point_t other, float count) const {
		return std::abs(this->x - other.x) <count && std::abs(this->y - other.y) < count;
	};

	double courseToDest(Point_t other) const {
			double alpha = 0;
			if (this->x == other.x) {
				if (other.y > this->y) {
					alpha = PI;
				} else if (other.y < this->y) {
					alpha = 3*PI/2;
				} else {
					alpha = 0;
				}
			} else {
				alpha = atan((other.y - this->y) / (other.x - this->x));

				if (other.x < this->x) {
					alpha += PI;
				}

				if (alpha < 0) {
					alpha += PI*2;
				}
			}
//			ROS_INFO_COND(DEBUG_ENABLE,"alpha = %d, th = %f (%f, %f)", alpha, this->th, this->x, this->y);
			return ranged_radian(alpha - this->th);
	}

	double radianDiff(const Point_t &other) const {
		return ranged_radian(this->th - other.th);
	}

	Cell_t toCell() const {
    return {countToCell(x), countToCell(y)};
  }

	bool isCellEqual(const Point_t &r) const
	{
		return  toCell() == r.toCell();
	}

	bool isRadianNear(const Point_t &r) const
	{
		return  std::abs(ranged_radian(th - r.th)) < degree_to_radian(20);
	}

	bool isCellAndAngleEqual(const Point_t &r) const
	{
//		ROS_ERROR("isCellEqual(%d),isRadianNear(%d) ", isCellEqual(r), isRadianNear(r));
		return  isCellEqual(r) && isRadianNear(r);
	}

	// in radian.
	double th{};
	Dir_t dir{};
private:
	int16_t countToCell(float count) const {
		return static_cast<int16_t>(round(count / CELL_SIZE));
  }

	float cellToCount(int16_t i) const {
		return i * CELL_SIZE;
	}

};

class Mark_t : public Vector2<int>{
public:
	Mark_t(){
		x = 0;
		y = 0;
		time = 0;
	}
	Mark_t(int _x,int _y,double _time){
		x = _x;
		y = _y;
		time = _time;
	}
	double time{};
};
typedef std::deque<Mark_t> Marks;
typedef struct
{
  double A;
  double B;
  double C;
  int len;
  double dis;
  double x1;//point 1
  double y1;
  double x2;//point 2
  double y2;
  double K; //gradient in degree
  double dist_2_this_line(Vector2<double> p){
	  return std::abs(A*p.x+B*p.y+C)/sqrt(A*A+B*B);
  }
} LineABC;

/*typedef */enum {
	// The sequence of CLEAN_MAP value must be UNCLEAN < CLEANED < MAP_BLOCKED < SLAM_MAP_BLOCKED
  UNCLEAN  = 0,
  SLAM_MAP_UNKNOWN = 0,
  CLEANED = 1,
  SLAM_MAP_CLEANABLE = 1,
  BLOCKED = 2,
  BLOCKED_FW = 2,
  BLOCKED_BUMPER = 3,
  BLOCKED_CLIFF = 4,
  BLOCKED_RCON = 5,
  BLOCKED_TMP_RCON = 6,
  BLOCKED_LIDAR = 7,
  BLOCKED_TILT = 8,
  BLOCKED_SLIP = 9,
  SLAM_MAP_BLOCKED = 10,
  BLOCKED_BOUNDARY = 11,
  TARGET_CLEAN = 13,
  TARGET = 14,
  COST_NO = 0,
  COST_1 = 1,
  COST_2 = 2,
  COST_3 = 3,
  COST_4 = 4,
  COST_5 = 5,
  COST_PATH = 6,
  COST_HIGH = 7,
};
typedef int CellState;

typedef std::pair<const CellState, Cell_t> PairCell_t;
class PointSelector{
public:
	PointSelector(bool is_left, double wall_distance);

	bool LaserPointRange(const Vector2<double> &point, bool is_corner) const;
	bool TargetPointRange(const Vector2<double> &target);
	bool inForwardRange(const Vector2<double> &point) const;
	bool inSidedRange(const Vector2<double> &point) const;

	double narrow;
	double narrow_minuend;
	bool is_left_;
	double x_min_forward;
	double x_max_forward;
	double x_min_side;
	double x_max_side;

	double y_min;
	double y_max;

	double y_min_forward;
	double y_max_forward;

	double y_min_side;
	double y_max_side;

	double y_min_point1_corner;
	double y_max_point1_corner;
	double y_min_point1;
	double y_max_point1;

	double y_min_target;
	double y_max_target;

	const double CHASE_X = 0.107;

	double corner_front_trig_lim;

	const int forward_count_lim = 10;
	const int side_count_lim = 20;
};

float two_points_distance_double(float startx,float starty,float destx,float desty);
void matrix_translate(double *x, double *y, double offset_x, double offset_y);
void matrix_rotate(double *x, double *y, double theta);

double line_angle(LineABC l, uint8_t mode);

void coordinate_transform(double *x, double *y, double theta, double offset_x, double offset_y);

/*
 * Function for formatting unsigned long integer to hex string.
 * @return: True for formatting succeeded. False for string too short to load the number.
 */
bool unsigned_long_to_hex_string(unsigned long number, char *str, const int len);

#endif
