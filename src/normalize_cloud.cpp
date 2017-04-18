/*
 * normalize_cloud.cpp
 *
 *  Created on: 01.02.2016
 *      Author: user
 */
#include <velodyne_pointcloud/rawdata.h>
#include "ATan.h"
#include "QTransform.h"
#include <vector>
#include <cfloat>

namespace registration {


typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;


//returns index of first ray in cloud
size_t normalize_cloud(const VPointCloud::ConstPtr& in,
                     const VPointCloud::Ptr& out,
                     const ATan& Atan,
                     size_t ring_number)
{
  out->resize(Atan.get_size()* ring_number);
  std::vector<double> ranges(out->size(), FLT_MAX);
  std::vector<int> indexes(out->size(), -1);
  //save index of last point
  const VPoint& pointn = in->back();
  size_t last_index = Atan.angle2index_cw(pointn.x, pointn.y);
  //index of first point
  const VPoint& point0 = in->front();
  size_t first_index = Atan.angle2index_cw(point0.x, point0.y);
  int delta = last_index - first_index;
  if ( delta < -(int)Atan.get_size()/2 )
    delta += Atan.get_size();
  //do not take into account old rays between first and last index
  delta = (0 <= delta)?
              (delta + 1)*ring_number : 0;

  //arrange points in to sectors
  for(int i = in->size()-1; i >= delta; i--)
  {
    const VPoint& point = in->operator [](i);
    double range = point.x*point.x + point.y*point.y + point.z * point.z;
    size_t index_in_ring = Atan.angle2index_cw(point.x, point.y);
//    if ( index_in_ring != Atan.angle2index_atan_cw(point.x, point.y))
//    {
//      ROS_INFO_STREAM(index_in_ring<<" "<<Atan.angle2index_atan_cw(point.x, point.y));
//    }
    size_t cloud_index = point.ring*Atan.get_size() + index_in_ring;
    assert(cloud_index < ranges.size());
    if ( range < ranges[cloud_index] )
    {
      ranges[cloud_index] = range;
      indexes[cloud_index] = i;
    }
  }
  //form normalized cloud
  for (size_t ring = 0; ring<ring_number; ring++)
  {
    size_t ind0 = ring * Atan.get_size();
    for(size_t i = 0; i<Atan.get_size(); i++)
    {
      size_t ind = ind0 + i;
      VPoint& point = out->operator [](ind);
      point.ring = ring;
      if ( indexes[ind] == -1 ) // no rays in sector
      {
        point.intensity = 0;
        point.x = 0;
        point.y = 0;
        point.z = 0;
      }
      else
      {  //copy point from input cloud
        const VPoint& in_point = in->operator [](indexes[ind]);
        point.intensity = in_point.intensity;
        point.x = in_point.x;
        point.y = in_point.y;
        point.z = in_point.z;
      }
    }
  }//end form cloud
  return (last_index + 1)%Atan.get_size();
}

//corrects cloud according to specified velocities
//assuming first_ray_index was received first in dt time before last
void correct_cloud(const VPointCloud::Ptr& normalized_cloud,
                   size_t ring_number,
                   const tf::Vector3& linear_velocity,
                   const tf::Vector3& angular_velocity,
                   double dt,
                   size_t first_ray_index = 0)
{
  size_t rays_number = normalized_cloud->size() / ring_number;
  //go through rays starting from first to end
  for ( size_t ray = first_ray_index, i = 0;
      i < rays_number;
      ray=(ray+1)%rays_number, i++)
  {
    //time from last ray to current
    double dti = -dt*(float)(rays_number- i)/(float)(rays_number);
    QTransform Ti(angular_velocity*dti, linear_velocity*dti);

    for ( size_t ring = 0; ring < ring_number; ring++)
    {
      assert(ring *rays_number + ray < normalized_cloud->size());
      VPoint& point = normalized_cloud->operator [](ring *rays_number + ray);
      if (point.x*point.x + point.y*point.y > 0.001)
      {
        tf::Vector3 corr_point = Ti(tf::Vector3( point.x,
                                                 point.y,
                                                 point.z ) );
        point.x = corr_point.x();
        point.y = corr_point.y();
        point.z = corr_point.z();
      }
    }
  }
}

}  // namespace registration


