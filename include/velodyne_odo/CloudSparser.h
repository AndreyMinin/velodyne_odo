/*
 * CloudSparser.h
 *
 *  Created on: 14.03.2016
 *      Author: user
 */

#ifndef CLOUDSPARSER_H_
#define CLOUDSPARSER_H_

#include "vcloud.h"
#include "Map3D.h"
#include <ros/ros.h>


namespace registration
{

class CloudSparser
{
protected:
  Map3D<double, uint8_t> map;
  double rmax2;
  double rmin2;
public:
  /*
   * sparse cloud in its coordinates
   */
  VPointCloud::Ptr sparse(const VPointCloud::ConstPtr& in)
  {
    map.zero();
    VPointCloud::Ptr out(new VPointCloud);
    out->header = in->header;
    out->reserve(in->size());
   // ROS_DEBUG_STREAM("reserve "<<out->points.capacity()<<" size = "<<in->size());
    for (VPointCloud::const_iterator it = in->begin(); it != in->end(); it++)
    {
      double rad2 = it->x * it->x + it->y * it->y ;

      if ( (rad2 < rmin2) || (rad2 > rmax2) )
        continue;
      uint8_t* val = map.getValPtr(it->x, it->y, it->z);
      if (!val)  //out of map
        continue;
      if (*val)   //already busy
        continue;
      *val = 1;  //mark cell
      out->push_back(*it);
    }
    return out;
  }

  /*
   * @brief sparse cloud in first cloud coordinates
   * @param in - input cloud
   * @param transform - transform from first cloud to this one
   * @return output cloud (new allocated)
   */
  VPointCloud::Ptr sparse(const VPointCloud::ConstPtr& in,
                          QTransform& transform)
  {
    map.zero();
    VPointCloud::Ptr out(new VPointCloud);
    out->header = in->header;
    out->reserve(in->size());
    for (VPointCloud::const_iterator it = in->begin(); it != in->end(); it++)
    {
      double rad2 = it->x * it->x + it->y * it->y;
      if (rad2 < rmin2)
        continue;

      tf::Vector3 v = transform(tf::Vector3(it->x, it->y, it->z));
      uint8_t* val = map.getValPtr(v.x(), v.y(), v.z());
      if (!val)  //out of map
        continue;
      if (*val)   //already busy
        continue;
      *val = 1;  //mark cell
      out->push_back(*it);
    }
    return out;
  }

  CloudSparser(double dx, double Xmin, double Ymin, double Zmin,
               double Xsize, double Ysize, double Zsize,
               double Rmax, double Rmin):
                 map(Xmin, Ymin, Zmin, Xsize, Ysize, Zsize, dx, 0),
                 rmax2(Rmax*Rmax),
                 rmin2(Rmin*Rmin)
  {
  }

};

} /* namespace registration */
#endif /* CLOUDSPARSER_H_ */
