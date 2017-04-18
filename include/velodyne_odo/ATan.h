/*
 * ATan.h
 *
 *  Created on: 01.02.2016
 *      Author: user
 */

#ifndef ATAN_H_
#define ATAN_H_

#include <vector>
#include <math.h>

//fast angle index for given tangens calculation
//without using atan function
class ATan
{
protected:
  double min_max_res;
  size_t size;
  //precalculated tangens values for angles at index
  std::vector<double> tans;
  //precalculated maximum and minimum indexes for search
  std::vector<unsigned int> mins, maxs;
  double resolution;
  inline size_t find(double tg, int min, int max ) const
  {
    int delta = max - min;
    while (delta > 1)
    {
      int mid = min + delta / 2;
      if (tg < tans[mid])
        max = mid;
      else
        min = mid;
      delta = max - min;
    }
    return min;
  }
  //x > 0 y >0
  inline size_t angle2index_abs(double x, double y) const
  {
    //x close to zero case
    if (x * tans.back() < y)
      return tans.size() - 1;
    //else
    double tangens = y / x;
    int tangens_index = tangens / min_max_res;

    return find(y / x, mins[tangens_index], maxs[tangens_index]);
  }


public:

  size_t get_size() const { return size; }
  double get_resolution() const { return resolution; }
  //functions with atan calculation only for checking
  size_t angle2index_atan_ccw(double x, double y) const
  {
    double angle = atan2(y, x);
    if (angle < 0)
      angle += 2.0 * M_PI;
    size_t res = floor(angle / resolution);
    if (res == size)
      res--;
    return res;
  }

  size_t angle2index_atan_cw(double x, double y) const
    {
      double angle = -atan2(y, x);
      if (angle < 0)
        angle += 2.0 * M_PI;
      size_t res = floor(angle / resolution);
      if (res == size)
        res--;
      return res;
    }

  //conter clockwise
  size_t angle2index_ccw(double x, double y) const
  {
    if (x > 0)
    {
      if (y > 0)
        return angle2index_abs(x, y);
      else
        return size - angle2index_abs(x, -y) - 1;
    }
    else //x < 0
    {
      if (y > 0)
        return size / 2 - angle2index_abs(-x, y) - 1;
      else
        return size / 2 + angle2index_abs(-x, -y);
    }
    return 0;
  }
  //clockwise
  size_t angle2index_cw(double x, double y) const
   {
     if (x > 0)
     {
       if (y < 0)
         return angle2index_abs(x, -y);
       else
         return size - angle2index_abs(x, y) - 1;
     }
     else //x < 0
     {
       if (y > 0)
         return size / 2 + angle2index_abs(-x, y);
       else
         return size / 2 - angle2index_abs(-x, -y) - 1;
     }
     return 0;
   }

  ATan(size_t _size = 360, double _min_max_res = 0.001 ):
    min_max_res(_min_max_res),
    size(_size),
    tans(size/4 ),
    resolution(M_PI*2.0/size)
  {
    for( size_t i = 0; i<tans.size(); i++)
    {
      tans[i] = tan(i*resolution);
    }

    double max_tan = tans.back();
    size_t estimates_size = max_tan/min_max_res + 1;

    maxs.resize(estimates_size);
    mins.resize(estimates_size);
    for( int i =0; i<estimates_size; i++)
    {
      //for tan in [i*min_max_res (i+1)*min_max_res]
      maxs[i] = ceil(atan((i+1)*min_max_res)/ resolution);
      if (maxs[i]>=tans.size())
        maxs[i] = tans.size()-1;
      mins[i] = floor(atan(i*min_max_res)/resolution);

    }

  }

};

#endif /* ATAN_H_ */
