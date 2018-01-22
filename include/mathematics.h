#ifndef __MYMATH_H
#define __MYMATH_H

#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <deque>
#include <ros/ros.h>
#include "config.h"

#define PI  3.141592653589793

#ifndef M_PI

#define M_PI	3.141592653589793

#endif

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

double ranged_angle(double angle);
double deg_to_rad(double deg, int8_t scale = 1);
//double rad_2_deg(double rad, int8_t scale);

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

class Point_t:public Vector2<float> {
public:
  Point_t() {
    x = 0;
    y = 0;
    th = 0;
  }

  Point_t(float _x, float _y, double _th) {
    x = _x;
    y = _y;
    th = _th;
  }

	Point_t addAngle(double diff) const {
		return {this->x, this->y, ranged_angle(this->th + diff)};
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
			ROS_INFO_COND(DEBUG_ENABLE,"alpha = %d, th = %d (%f, %f)", alpha, this->th, this->x, this->y);
			return ranged_angle(alpha - this->th);
	}

	double angleDiff(const Point_t& other) const {
		return ranged_angle(this->th - other.th);
	}

  Cell_t toCell() const {
    return {countToCell(x), countToCell(y)};
  }

	bool isCellEqual(const Point_t &r) const
	{
		return  toCell() == r.toCell();
	}

	bool isAngleNear(const Point_t &r) const
	{
		return  std::abs(ranged_angle(th - r.th)) < 20*PI/180;
	}

	bool isCellAndAngleEqual(const Point_t &r) const
	{
		return  isCellEqual(r) && isAngleNear(r);
	}

  double th{};
private:
  int16_t countToCell(float count) const {
		return static_cast<int16_t>(round(count / CELL_SIZE));
  }

	float cellToCount(int16_t i) const {
		return i * CELL_SIZE;
	}

};

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

float two_points_distance_double(float startx,float starty,float destx,float desty);
void matrix_translate(double *x, double *y, double offset_x, double offset_y);
void matrix_rotate(double *x, double *y, double theta);

double line_angle(LineABC l, uint8_t mode);

void coordinate_transform(double *x, double *y, double theta, double offset_x, double offset_y);


#endif
