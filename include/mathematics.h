#ifndef __MYMATH_H
#define __MYMATH_H

#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <ostream>
#include <bits/unique_ptr.h>
#include <functional>
#include <algorithm>
#include <assert.h>
#include "config.h"

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

float cellToCount(int16_t distance);

//int16_t countToCell(int32_t count);

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
    /*
    * @brief cal mod
    */
    inline int operator%(int val)const
    {
        return static_cast<int>((x*y) % val);
    } 
    inline bool operator>(const Vector2& rOther) const
    {
        Vector2 this_{x,y};
        return !(this_ < rOther || this_ == rOther);
    }
    /**
     * Write vector onto output stream
     */
    friend std::ostream& operator<<(std::ostream& rStream, const Vector2& rVector)
    {
      rStream << "{" << rVector.GetX() << "," << rVector.GetY() << "}";
      return rStream;
    }

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

using Dir_t = int;
enum {
	X_Y_LANE=0,
	CURR_LANE,
    CURR_LANE_NEG,
    Y_AXIS_POS_NEXT,
    Y_AXIS_POS,
    Y_AXIS_NEG_NEXT,
	XY_AXIS_ANY,
    RANGE_END,
};

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
	Point_t(const Vector2<float> p) {
    x = p.x;
    y = p.y;
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

	Point_t getCenterRelative(float dx, float dy) const {
		auto Cell = this->toCell();
		Point_t point{cellToCount(Cell.x),cellToCount(Cell.y),th};
		return point.getRelative(dx,dy);
	}

	bool isNearTo(Point_t other, float count) const {
//		return std::abs(this->x - other.x) <count && std::abs(this->y - other.y) < count;
		return sqrt(pow(this->x - other.x, 2) + pow(this->y - other.y, 2)) < count;
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

	Point_t project(const Point_t &p1, const Point_t &p2) const
	{
		float cross = (p2.x - p1.x) * (x - p1.x) + (p2.y - p1.y) * (y - p1.y);
		float d2 = (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);
		float r = cross / d2;
		Point_t p;
		p.x = p1.x + (p2.x - p1.x) * r;
		p.y = p1.y + (p2.y - p1.y) * r;
		return p;
	}

	float project_ratio(const Point_t& p1, const Point_t& p2) const
	{
		auto cross = (p2.x - p1.x) * (x - p1.x) + (p2.y - p1.y) * (y - p1.y);

		assert(p2 != p1);

		auto d2 = (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);

		return cross / d2;
	}

	float distance(const Point_t& p1, const Point_t& p2) const
	{
		auto r = project_ratio(p1, p2);
		if(r <= 0)
			return this->Distance(p1);
		if(r >= 1)
			return this->Distance(p2);

		auto pj = p1 + (p2-p1) * r;

		return this->Distance(pj);
	}
	// in radian.
	double th{};
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
	// Note: don't change MAP_BLOCKED order, c_block has priority by this order --linshaoyue
  UNCLEAN  = 0,
  SLAM_MAP_UNKNOWN = 0,
  CLEANED = 1,
  SLAM_MAP_REACHABLE = 1,
  BLOCKED = 2,
  BLOCKED_FW = 2,
  BLOCKED_BUMPER = 3,
	BLOCKED_LIDAR = 4,
  BLOCKED_CLIFF = 5,
  BLOCKED_RCON = 6,
  BLOCKED_TMP_RCON = 7,
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
//sensor_msgs::LaserScan
template <typename T>
class DequeArray {
public:
	typedef typename std::deque<T>::iterator iterator;
	typedef typename std::deque<T>::const_iterator const_iterator;
    DequeArray(int size):valid_size_(size){ };
	void push_back(T i) {
		d.push_back(i);
		if (d.size() > valid_size_)
			d.pop_front();
	}
	void push_front(T i) {
		d.push_front(i);
		if (d.size() > valid_size_)
			d.pop_back();
	}

	void pop_front() {
		d.pop_front();
	}
	bool empty()
	{
		return d.size() == 0;
	}

	iterator erase(const iterator& it)
	{
		return d.erase(it);
	}

	T& front() {
		return d.front();
	}

	T& back() {
		return d.back();
	}

	typename std::deque<T>::iterator begin() {
		return d.begin();
	}

	typename std::deque<T>::iterator end() {
		return d.end();
	}

	size_t size() {
		return d.size();
	}

	void clear() {
		d.clear();
	}

	T operator[](int i) {
		return d[i];
	}
//	using iterator =  std::deque::const_iterator;
	bool is_full()
	{
		return valid_size_ == d.size();
	}

private:
	std::deque<T> d;
    int valid_size_{3};
};

using HomePoints_t=DequeArray<Point_t>;
class HomePointsManager
{
public:
	HomePointsManager();

	void for_each(const std::function<void(const Point_t& it)>& lambda_fun) ;

	void resetRconPoint();

	void setStartPointRad(double th);

	double getStartPointRad();

	HomePoints_t::iterator getStartPoint();

	bool isStartPoint();

	void popCurrRconPoint();

	void setRconPoint(const Point_t &point);

	typename std::deque<HomePoints_t>::iterator end()
	{
		return home_points_list_.end();
	}

	typename std::deque<HomePoints_t>::iterator begin()
	{
		return home_points_list_.begin();
	}

	typename std::deque<HomePoints_t>& home_points_list()
	{
		return home_points_list_;
	}

	typename std::deque<HomePoints_t>::iterator& home_points_list_it()
	{
		return home_points_list_it_;
	}

	HomePoints_t::iterator& home_point_it()
	{
		return home_point_it_;
	}


private:
	std::deque<HomePoints_t> home_points_list_{HomePoints_t(3), HomePoints_t(1)};
	std::deque<HomePoints_t>::iterator home_points_list_it_;
	HomePoints_t::iterator home_point_it_;

};

typedef std::pair<const CellState, Cell_t> PairCell_t;
typedef std::deque<Point_t> Points;

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

bool isAny(Dir_t dir);


Vector2<double> polarToCartesian(double polar, int i);


class CellEqual
{
public:
	CellEqual(const Cell_t& cell):cell_(cell){ };
	bool operator()(const std::pair<Cell_t, int> & c_it)
	{
		return cell_ == c_it.first;
	}
	bool operator()(const Cell_t& c_it)
	{
		return cell_ == c_it;
	}
private:
	Cell_t cell_;
};

//Dir_t get_dir(const Cells::iterator& neighbor, const Cells::iterator& curr);
//Dir_t get_dir(const Cell_t& neighbor, const Cell_t& curr);
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

std::unique_ptr<Cells> points_to_cells(const Points& points);
std::unique_ptr<Points> cells_to_points(const Cells& path);

void displayCellPath(const Cells &path);
void displayPointPath(const Points &point_path);
void displayTargetList(const Cells &target_list);
#endif
