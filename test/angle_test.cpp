//
// Created by lsy563193 on 17-3-7.
//
#include <obstacle_detector/SegmentObstacle.h>
#include "Angles.h"
using namespace std;
using namespace obstacle_detector;

int main(int argc, char *argv[])
{
//  Segment s1(Point(20, 20), Point(20,-20));
//  Segment s2(Point(20.04, 20), Point(20,-20));
//  Segment s3(Point(20.02, 20), Point(20,-20));

  Segment s1(Point(20, 20), Point(20,-20));
  Segment s2(Point(20, 20), Point(20,-20.02));
  Segment s3(Point(20, 20), Point(20,-20.04));
  Segment inputs[] = {s1,s2,s3};
  Segment_set segment_ss;
  for (auto& seg:inputs)
    segment_ss.classify(seg);

  auto seg = segment_ss.min_distant_segment();
  cout << "min_seg: " << seg <<endl;

}