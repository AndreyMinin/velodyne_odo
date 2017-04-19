/*
 * CloudViewer.h
 *
 *  Created on: 8 нояб. 2016 г.
 *      Author: user
 */

#ifndef TEST_CLOUD_VIEWER_H_
#define TEST_CLOUD_VIEWER_H_
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "viewer.h"
#include <queue>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>


typedef boost::mutex bmutex;
typedef boost::unique_lock<bmutex> block;

class CloudViewer :public Viewer
{
protected:
  tf::TransformListener tfListener;
  std::string map_frame;
  int tms;
  double r, g, b;
  double grid_size;
  double rad;
  double max_rad;
  double min_rad;
  double min_z;
  double max_z;
  ros::Subscriber laser_sub;
  bool stopped;
  bmutex mutex;
  std::queue<VPointCloud::Ptr> cloud_que;
  //condition for range filter
  pcl::ConditionAnd<VPoint>::Ptr range_cond;

  pcl::ConditionalRemoval<VPoint> cond_rem;

  pcl::VoxelGrid< VPoint > grid;

  void build_range_filter();

  void build_grid_filter();

  void range_filter(const VPointCloud::ConstPtr& input,
                       VPointCloud::Ptr& out);

  void simple_filter_pcloud(const VPointCloud::ConstPtr& input,
                     VPointCloud::Ptr& out);
  void grid_filter(const VPointCloud::ConstPtr& input,
                       VPointCloud::Ptr& out);

  void filter(const VPointCloud::ConstPtr& input,
                         VPointCloud::Ptr& out);

  void processCloud(const VPointCloud::ConstPtr &scan);

  void viewer_thread();
public:
  CloudViewer(ros::NodeHandle& nh);
  virtual ~CloudViewer();
  void run();
};

#endif /* TEST_CLOUD_VIEWER_H_ */
