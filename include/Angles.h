//
// Created by lsy563193 on 17-3-7.
//

#ifndef PP_ANGLES_H
#define PP_ANGLES_H

#include <vector>
#include <iostream>
template <class T>
class Angles
{
public:
  void classify(T t);
  int16_t max_distant_angle();
  void clear();
//  T longest();
private:
  std::vector<std::vector<T>> angles_set;
  int16_t average(std::vector<T> L,bool n);
//  T average(std::vector<T> L);
};

template <class T>
int16_t Angles<T>::average(std::vector<T> L,bool n)
{
  int16_t sum{};

  for(auto l:L){
    if(n == false)
      sum += l.first;
    else{
      sum += l.second;
    }
  }

  int16_t avr = sum / (int16_t)L.size();

  return avr;
}

template <class T>
void Angles<T>::classify(T pair)
{
  auto pair_iter = angles_set.begin();

  for (pair_iter; pair_iter != angles_set.end(); ++pair_iter)
  {
    std::vector<T> v(*pair_iter);
    if (::abs(pair.first - average(v,false)) < 50)
    {
      (*pair_iter).push_back(pair);
      break;
    }
  }
  if (pair_iter == angles_set.end())
  {
    std::vector<T> pairs;
    pairs.push_back(pair);
    angles_set.push_back(pairs);
  }
}

template <class T>
int16_t Angles<T>::max_distant_angle()
{
  int16_t max_angle{}; double max_dist{};
  auto best_val = std::make_pair(max_angle,max_dist);
  std::cout << "-----Angles display------" << std::endl;
  for (auto angles:angles_set)
  {
    std::cout << "--------------------" << std::endl;

    double sum{},dist;
    int16_t angle{};
    for (auto angle:angles)
    {
      std::cout << "angle = " << angle.first << std::endl;
      std::cout << "dist = " << angle.second << std::endl;
    }
    angle = average(angles,false);
    for(auto angle_:angles){
      sum += angle_.second;
    }
    dist = sum/angles.size();
    //----------------------------
    if(dist > best_val.second){
      best_val.first = angle;
      best_val.second = dist;
    }
    std::cout << "angle average:" << angle << std::endl;
    std::cout << "dist average:" << dist << std::endl;
    std::cout << "max_angle average:" << best_val.first << std::endl;
    std::cout << "max_distant average:" << best_val.second << std::endl;
    std::cout << "$$$$$$$$$$$$$$$$$$$$$" << std::endl;
    return best_val.first;
  }
  return 0;
}

template <class T>
void Angles<T>::clear()
{
  angles_set.clear();
}

//template <class T>
//T Angles<T>:: longest()
//{
//  for(auto angle:angles_set){
//    if(angle.size()>3){
//      if(angle.)
//    }
//  }
//}
#endif //PP_ANGLES_H
