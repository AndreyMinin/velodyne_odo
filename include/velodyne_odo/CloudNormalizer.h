/*
 * CloudNormalizer.h
 *
 *  Created on: 28.04.2016
 *      Author: user
 */

#ifndef CLOUDNORMALIZER_H_
#define CLOUDNORMALIZER_H_

#include "vcloud.h"
#include "ATan.h"
#include "QTransform.h"
#include <vector>
#include <cfloat>

namespace registration
{
//class performs normalization and correction of a velodyne point cloud
//after normalization number of rays is fixed to rays x rings
//and each ray angle is correspond to its index in cloud
//during correction each ray of normalized cloud is corrected
//according to linear velocity and angular rate
class CloudNormalizer
{
protected:
  size_t ring_number;
  ATan Atan;
  //array with indexes of rays in normalized cloud
  std::vector<int> normal_index;
  //mask for corrupted rays
  std::vector<int> corrupted_mask;
  //fills index of first ray in cloud
  //returns number of valid rays
  size_t  normalize_cloud(const VPointCloud::ConstPtr& in,
                          const VPointCloud::Ptr& out,
                          size_t& first_ray_index);
  //corrects cloud according to specified velocities
  //assuming first_ray_index was received first in dt time before last
  void correct_cloud(const VPointCloud::Ptr& normalized_cloud,
                     const tf::Vector3& linear_velocity,
                     const tf::Vector3& angular_velocity,
                     double dt,
                     size_t first_ray_index);
  void fill_corrupted_mask(const std::vector<double>& start_mask_angles,
                 const std::vector<double>& finish_mask_angles);
public:
  //returns untwisted cloud of rays*rings size
  //non valid points fields are set to zeros
  //returns number of valid rays (as normalize_cloud)
  size_t normalize(const VPointCloud::Ptr& out,
                    const VPointCloud::ConstPtr& in,
                    const tf::Vector3& linear_velocity,
                    const tf::Vector3& angular_velocity,
                    double dt);


  //version with corrupted mask - angles in degrees, counter clockwise in velodyne SK
  CloudNormalizer(size_t rings, size_t rays,
                  const std::vector<double>& start_mask_angles,
                  const std::vector<double>& finish_mask_angles,
                  double _min_max_res = 0.001):
    ring_number(rings),
    Atan(rays, _min_max_res),
    normal_index(Atan.get_size()* ring_number),
    corrupted_mask(Atan.get_size(), 0)
  {
    if ( !start_mask_angles.empty() && !finish_mask_angles.empty() )
      fill_corrupted_mask(start_mask_angles, finish_mask_angles);
  }
  //version without corrupted
  CloudNormalizer(size_t rings, size_t rays,
                   double _min_max_res = 0.001):
     ring_number(rings),
     Atan(rays, _min_max_res),
     normal_index(Atan.get_size()* ring_number),
     corrupted_mask(Atan.get_size(), 0)
  {}
};

} /* namespace registration */
#endif /* CLOUDNORMALIZER_H_ */
