/*
 * vcloud.h
 *
 *  Created on: 14.03.2016
 *      Author: user
 */

#ifndef VCLOUD_H_
#define VCLOUD_H_
#include <velodyne_pointcloud/rawdata.h>

namespace registration {

// shorter names for point cloud types in this namespace
typedef velodyne_rawdata::VPoint VPoint;
typedef velodyne_rawdata::VPointCloud VPointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> RGBCloud;


}  // namespace registration



#endif /* VCLOUD_H_ */
