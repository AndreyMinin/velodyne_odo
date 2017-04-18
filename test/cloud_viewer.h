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
  double min_z;
  double max_z;
  ros::Subscriber laser_sub;
  bool stopped;
  bmutex mutex;
  std::queue<VPointCloud::Ptr> cloud_que;
  void simple_filter_pcloud(const VPointCloud::ConstPtr& input,
                     VPointCloud::Ptr& out);
  void filter_pcloud(const VPointCloud::ConstPtr& input,
                       VPointCloud::Ptr& out);

  void processCloud(const VPointCloud::ConstPtr &scan);

  void viewer_thread();
public:
  CloudViewer(ros::NodeHandle& nh);
  virtual ~CloudViewer();
  void run();
};

#endif /* TEST_CLOUD_VIEWER_H_ */
