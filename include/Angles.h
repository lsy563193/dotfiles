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
  void classive(T t);
  void display();
  void clear();
  T longest();
private:
  std::vector<std::vector<T>> angles_set;
  int16_t average(std::vector<T> L,bool n=false);
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
void Angles<T>::classive(T pair)
{
  auto pair_iter = angles_set.begin();

  for (pair_iter; pair_iter != angles_set.end(); ++pair_iter)
  {
    std::vector<T> v(*pair_iter);
    if (abs(pair.first - average(v)) < 10)
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
void Angles<T>::display()
{
  std::cout << "-----Angles display------" << std::endl;
  for (auto angles:angles_set)
  {
    std::cout << "--------------------" << std::endl;
    for (auto angle:angles)
    {
      std::cout << "angle = " << angle.first << std::endl;
      std::cout << "dist = " << angle.second << std::endl;
    }
    std::cout << "angle average:" << average(angles) << std::endl;
    std::cout << "distant average:" << average(angles,true) << std::endl;
    std::cout << "$$$$$$$$$$$$$$$$$$$$$" << std::endl;
  }
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
