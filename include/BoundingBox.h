//
// Created by lsy563193 on 8/7/17.
//

#ifndef PP_BOUNDINGBOX_H
#define PP_BOUNDINGBOX_H


/**
   * Defines a 2-dimensional bounding box.
   */

class BoundingBox2 {
  class iterator {
  public:
    iterator(Cell_t *pos,int16_t min, int16_t max): pos_(pos),min_(min),max_(max)
    {
//      ROS_INFO("pos %d ", pos_);
    }

    Cell_t operator*() const
    {
      return *pos_;
    }

    bool operator!=(const iterator &other) const
    {
//      ROS_INFO("this(%d,%d),other(%d,%d)", pos_->x,pos_->y, other.pos_->x, other.pos_->y);
      return *pos_ != *other.pos_;
    }

    const iterator &operator++()
    {
      pos_->x++;
      if(pos_->x > max_){
        pos_->x = min_;
        pos_->y++;
      }
      return *this;
    }

  private:
    Cell_t *pos_;
    int16_t min_;
    int16_t max_;
  };

public:
  /*
	 * Bounding box of maximal size
	 */
  BoundingBox2()
          : min(INT16_MAX, INT16_MAX), max(INT16_MIN, INT16_MIN)
  {pos_ = min; };

  /**
	 * Bounding box with given minimum and maximum values.
	 * @param rMinimum minimum value
	 * @param rMaximum maximum value
	 */
  BoundingBox2(const Cell_t &rMinimum, const Cell_t &rMaximum)
          : min(rMinimum), max(rMaximum)
  {pos_ = min;};

public:
  /**
	 * Get bounding box minimum
	 * @return bounding box minimum
	 */
  inline const Cell_t &GetMinimum() const
  {
    return min;
  }

  /**
	 * Set bounding box minimum
	 * @param rMinimum bounding box minimum
	 */
  inline void SetMinimum(const Cell_t &rMinimum)
  {
    min = rMinimum;
  }

  /**
	 * Get bounding box maximum
	 * @return bounding box maximum
	 */
  inline const Cell_t &GetMaximum() const
  {
    return max;
  }

  /**
	 * Set bounding box maximum
	 * @param rMaximum bounding box maximum
	 */
  inline void SetMaximum(const Cell_t &rMaximum)
  {
    max = rMaximum;
  }

  /**
	 * Get the size of this bounding box
	 * @return bounding box size
	 */
  /* inline Size2 <int16_t> GetSize() const
	 {
		 Cell_t size = max - min;

		 return Size2<int16_t>(size.GetX(), size.GetY());
	 }*/

  /**
	 * Add vector to bounding box
	 * @param rPoint point
	 */
  inline void Add(const Cell_t &rPoint)
  {
    min.MakeFloor(rPoint);
    max.MakeCeil(rPoint);
  }

  /**
	 * Add given bounding box to this bounding box
	 * @param rBoundingBox bounding box
	 */
  inline void Add(const BoundingBox2 &rBoundingBox)
  {
    Add(rBoundingBox.GetMinimum());
    Add(rBoundingBox.GetMaximum());
  }

  /**
	 * Whether the given point is in the bounds of this box
	 * @param rPoint point
	 * @return whether the given point is in the bounds of this box
	 */
/*  inline  Contains(const Cell_t &rPoint) const
  {
    return (math::InRange(rPoint.GetX(), min.GetX(), max.GetX()) &&
            math::InRange(rPoint.GetY(), min.GetY(), max.GetY()));
  }*/

  /**
	 * Checks if this bounding box intersects with the given bounding box
	 * @param rOther bounding box
	 * @return true if this bounding box intersects with the given bounding box
	 */
  inline bool Intersects(const BoundingBox2 &rOther) const
  {
    if ((max.GetX() < rOther.min.GetX()) || (min.GetX() > rOther.max.GetX()))
    {
      return false;
    }

    if ((max.GetY() < rOther.min.GetY()) || (min.GetY() > rOther.max.GetY()))
    {
      return false;
    }

    return true;
  }

  /**
	 * Checks if this bounding box contains the given bounding box
	 * @param rOther bounding box
	 * @return true if this bounding box contains the given bounding box
	 */
  inline bool Contains(const BoundingBox2 &rOther) const
  {
    if ((max.GetX() < rOther.min.GetX()) || (min.GetX() > rOther.max.GetX()))
    {
      return false;
    }

    if ((max.GetY() < rOther.min.GetY()) || (min.GetY() > rOther.max.GetY()))
    {
      return false;
    }

    if ((min.GetX() <= rOther.min.GetX()) && (rOther.max.GetX() <= max.GetX()) &&
        (min.GetY() <= rOther.min.GetY()) && (rOther.max.GetY() <= max.GetY()))
    {
      return true;
    }

    return false;
  }

  iterator begin()
  {
    iterator sc(&pos_,min.x,max.x);
//    ROS_INFO("min(%d)",sc);
    return sc;
  }

  iterator end()
  {
		end_ = {min.x,int16_t(max.y+1)};
    iterator sc(&end_,min.x,max.x);
//    iterator sc(&max);
//    ROS_INFO("max(%d)",sc);
    return sc;
  }

public:
  Cell_t min;
  Cell_t max;
  Cell_t end_;
  Cell_t pos_;
}; // class BoundingBox2

#endif //PP_BOUNDINGBOX_H
