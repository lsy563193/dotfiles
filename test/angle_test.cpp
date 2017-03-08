//
// Created by lsy563193 on 17-3-7.
//
#include "Angles.h"
using namespace std;

typedef double Angle;

int main(int argc, char *argv[])
{
//  std::vector inputs{1*//*,2,3 ,11, 21, 23, 44, 56, 66, 3, 53, 45*//*};
  Angles<std::pair<double,double>> angles;
//  angles_set.clear();

  std::vector<std::pair<double,double>> inputs{make_pair(1,3),make_pair(1,4),make_pair(15,5),make_pair(11,2)};
  for (auto data:inputs)
    angles.classive(data);

  angles.display();

}