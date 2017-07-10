//
// Created by lsy563193 on 17-3-7.
//

#ifndef PP_ANGLES_H
#define PP_ANGLES_H

#include <vector>
#include <iostream>
#include <figures/segment.h>
using namespace obstacle_detector;

class Segment_set
{
public:
  void classify(Segment& t);
  double min_distant_segment_angle();
  void clear();
private:
  std::vector<std::vector<Segment>> segmentss;
};

Point average(std::vector<Point> P)
{
  Point sum(0, 0);

  for(Point& p : P){
    sum += p;
  }

  Point avr = sum / (int16_t)P.size();

  return avr;
}
std::vector<Point> get_first(const std::vector<Segment> &segments)
{
  std::vector<Point> v_first;

  auto segment_it = segments.begin();
  for (; segment_it != segments.end(); ++segment_it)
    v_first.push_back((*segment_it).first_point);

  return v_first;
}

std::vector<Point> get_last(const std::vector<Segment> &segments)
{
  std::vector<Point> v_last;

  auto segment_it = segments.begin();
  for (; segment_it != segments.end(); ++segment_it)
    v_last.push_back((*segment_it).last_point);

  return v_last;
}

void Segment_set::classify(Segment& seg)
{
  auto segments_it = segmentss.begin();

  for (segments_it; segments_it != segmentss.end(); ++segments_it)
  {
    std::vector<Point> v_first, v_last;
    std::vector<Segment> segments(*segments_it);

    v_first = get_first(segments);
    v_last = get_last(segments);

    if ( (seg.first_point - average(v_first)).length() < 0.2 &&
         (seg.last_point - average(v_last)).length() < 0.2 )
    {
      (*segments_it).push_back(seg);
      break;
    }
  }
  if (segments_it == segmentss.end())
  {
    std::vector<Segment> segs;
    segs.push_back(seg);
    segmentss.push_back(segs);
  }
}

double Segment_set::min_distant_segment_angle()
{
  ROS_INFO("-----Segment_set display------");
  auto min_dist = std::numeric_limits<double>::max();
  auto index = 0;
  auto it = 0;

  for (auto& segments : segmentss)
  {
    if(segments.size()<5)
      continue;
    ROS_INFO("--------------------");
    ROS_INFO("segments: ");
    for (auto& segment : segments)
      ROS_INFO("(%f,%f),(%f,%f)",segment.first_point.x, segment.first_point.y, segment.last_point.x, segment.last_point.y);

    auto seg_arv = Segment( average(get_first(segments)),average(get_last(segments)) );
    ROS_INFO("seg_arv: ");
    double dist = seg_arv.distanceTo(Point(0,0));
    ROS_INFO("average dist:");
    if(dist < min_dist){
      min_dist = dist;
      ROS_INFO("min_dist:");
      index = it;
    }
    it++;
  }
  auto angle = 0.0;
  if(it != 0)
    angle = ( average(get_last(segmentss[index])) - average(get_first(segmentss[index])) ).angleDeg();
  return angle;
}

void Segment_set::clear()
{
  segmentss.clear();
}

#endif //PP_ANGLES_H
