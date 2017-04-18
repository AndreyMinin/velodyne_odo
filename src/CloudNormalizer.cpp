/*
 * CloudNormalizer.cpp
 *
 *  Created on: 28.04.2016
 *      Author: user
 */

#include "CloudNormalizer.h"
#include <ros/ros.h>

namespace registration
{

void CloudNormalizer::fill_corrupted_mask(const std::vector<double>& start_mask_angles,
                 const std::vector<double>& finish_mask_angles)
{

  int size = std::min(start_mask_angles.size(), finish_mask_angles.size());
  //ROS_INFO_STREAM("fill_corrupted_mask "<<size);
  for( int i = 0; i<size; i++)
  {
    int start_index = floor( (360.0 - finish_mask_angles[i]) *M_PI/ 180.0 /Atan.get_resolution() );
    ROS_DEBUG_STREAM("start mask angle "<<360.0 - finish_mask_angles[i]<<" "<<start_index);
    if ( start_index < 0 )
    {
      ROS_WARN_STREAM("wrong mask start angle at index "<<i);
      continue;
    }
    int end_index = ceil( (360.0 - start_mask_angles[i]) *M_PI/ 180.0/ Atan.get_resolution() );
    ROS_DEBUG_STREAM("finish mask angle "<<360.0 - start_mask_angles[i]<<" "<<end_index);
    if ( end_index > Atan.get_size())
    {
      ROS_WARN_STREAM("wrong mask finish angle at index "<<i);
      continue;
    }
    for( int m = start_index; m<= end_index; m++)
    {
      corrupted_mask[m] = 1;
    }
  }
}

size_t  CloudNormalizer::normalize_cloud(const VPointCloud::ConstPtr& in,
                                         const VPointCloud::Ptr& out,
                                         size_t& first_ray_index)
{
  out->resize(Atan.get_size()* ring_number);
  std::vector<double> ranges(out->size(), FLT_MAX);
  //prepare mask
  std::fill(normal_index.begin(), normal_index.end(), -1);

  //save index of last point - newest point
  const VPoint& last_point = in->back();
  size_t last_index = Atan.angle2index_cw(last_point.x, last_point.y);

  //do not take into account old rays between first and last index
  size_t point_index = 0;

  for(; point_index < in->size(); point_index++)
  {
    const VPoint& point = in->operator [](point_index);
    size_t index_in_ring = Atan.angle2index_cw(point.x, point.y);
    int index_delta = last_index - index_in_ring;
    if ( index_delta < -(int)Atan.get_size()/2 )
      index_delta += Atan.get_size();
    if ( index_delta < 0 ) //get out of overlapping region
      break;
  }
  //ROS_DEBUG_STREAM("point delta = "<<point_index);
  //arrange points in to sectors
  for(int i = point_index; i <in->size(); i++)
  {
    const VPoint& point = in->operator [](i);
    double range = point.x*point.x + point.y*point.y + point.z * point.z;
    size_t index_in_ring = Atan.angle2index_cw(point.x, point.y);
    if ( corrupted_mask[index_in_ring] )
    {
      continue;
    }
    size_t normalized_index = point.ring*Atan.get_size() + index_in_ring;
    assert(normalized_index < ranges.size());
    if ( range < ranges[normalized_index] )
    {
      ranges[normalized_index] = range;
      normal_index[normalized_index] = i;
    }
  }
  size_t output_size = 0;
  //form normalized cloud
  for (size_t ring = 0; ring<ring_number; ring++)
  {
    size_t ind0 = ring * Atan.get_size();
    for(size_t i = 0; i<Atan.get_size(); i++)
    {
      size_t ind = ind0 + i;
      VPoint& point = out->operator [](ind);
      point.ring = ring;
      if ( normal_index[ind] == -1 ) // no rays in sector or ray is corrupted
      {
        point.intensity = 0;
        point.x = 0;
        point.y = 0;
        point.z = 0;
      }
      else
      {  //copy point from input cloud
        const VPoint& in_point = in->operator [](normal_index[ind]);
        point.intensity = in_point.intensity;
        point.x = in_point.x;
        point.y = in_point.y;
        point.z = in_point.z;
        output_size ++;
      }
    }
  }//end form cloud

  first_ray_index = (last_index + 1)%Atan.get_size();

  return output_size;
}

void CloudNormalizer::correct_cloud(const VPointCloud::Ptr& normalized_cloud,
                   const tf::Vector3& linear_velocity,
                   const tf::Vector3& angular_velocity,
                   double dt,
                   size_t first_ray_index)
{
  size_t rays_number = Atan.get_size();
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
      //point index in normalized cloud
      size_t point_index = ring *rays_number + ray;
      assert( point_index < normalized_cloud->size());

      VPoint& point = normalized_cloud->operator [](point_index);
      //if (point.x*point.x + point.y*point.y > 0.001)
      if ( normal_index[point_index] >= 0) //point valid
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

void test(int i, const VPointCloud::Ptr& out )
{
  VPoint& p = out->operator [](i);
  ROS_DEBUG_STREAM("test "<<i<<" "<<p.x<<" "<<p.y<<" "<<p.z);
}

size_t CloudNormalizer::normalize(
                          const VPointCloud::Ptr& out,
                           const VPointCloud::ConstPtr& in,
                           const tf::Vector3& linear_velocity,
                           const tf::Vector3& angular_velocity,
                           double dt)
{
  out->header = in->header;
  size_t first_ray_index;
  size_t normal_size = normalize_cloud( in,out, first_ray_index);

  correct_cloud(out, linear_velocity, angular_velocity, dt, first_ray_index );

//  test(0, out);
//  test(out->size()/2, out);
//  test(out->size()-1, out);
  return normal_size;
}

} /* namespace registration */
