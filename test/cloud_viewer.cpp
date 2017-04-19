/*
 * CloudViewer.cpp
 *
 *  Created on: 8 нояб. 2016 г.
 *      Author: user
 */

#include "cloud_viewer.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>

void CloudViewer::simple_filter_pcloud(const VPointCloud::ConstPtr& input, VPointCloud::Ptr& out)
{
  out->reserve(input->size());
  double max2 = max_rad * max_rad;
  double min2 = min_rad * min_rad;
  for (int i = 0; i < input->size(); ++i)
  {
    const VPoint& p = input->points[i];

    double range2 = p.x * p.x + p.y * p.y;
    if (range2 > max2 || range2 < min2)
      continue;

    if (p.z > max_z || p.z < min_z)
      continue;
    out->push_back(p);
  }
}

void CloudViewer::build_grid_filter()
{
  grid.setLeafSize(grid_size, grid_size, grid_size);
}

void CloudViewer::grid_filter(const VPointCloud::ConstPtr& input, VPointCloud::Ptr& output)
{
    grid.setInputCloud(input);
    grid.filter(*output);

}

void CloudViewer::filter(const VPointCloud::ConstPtr& input,
                        VPointCloud::Ptr& out)
{
  range_filter(input, out);
  if ( grid_size > 0 )
  {
    VPointCloud::Ptr grid_scan = boost::make_shared<VPointCloud>();
    grid_filter(out, grid_scan);
    out = grid_scan;
  }
  ROS_DEBUG_STREAM("input size = "<<input->size()<<" filtered size = "<<out->size());
}

void CloudViewer::build_range_filter()
{
  range_cond = boost::make_shared<pcl::ConditionAnd<VPoint> >();
//  range_cond->addComparison(boost::make_shared<pcl::FieldComparison<VPoint> >("z", pcl::ComparisonOps::GT, min_z));
//  range_cond->addComparison(boost::make_shared<pcl::FieldComparison<VPoint> >("z", pcl::ComparisonOps::LT, max_z));
  Eigen::Matrix3f A;
  A << 1, 0, 0,
       0, 1, 0,
       0, 0, 0;
  Eigen::Vector3f v = Eigen::Vector3f::Zero();
  //x^2 + y^2 < max_range^2
  range_cond->addComparison(
      boost::make_shared<pcl::TfQuadraticXYZComparison<VPoint> >(pcl::ComparisonOps::LT, boost::ref(A), boost::ref(v),
                                                                 -max_rad * max_rad));
  //x^2 + y^2 > min_range^2
  range_cond->addComparison(
      boost::make_shared<pcl::TfQuadraticXYZComparison<VPoint> >(pcl::ComparisonOps::GT, boost::ref(A), boost::ref(v),
                                                                 -min_rad * min_rad));

  cond_rem.setCondition(range_cond);
}

void CloudViewer::range_filter(const VPointCloud::ConstPtr& input, VPointCloud::Ptr& out)
{

  cond_rem.setInputCloud(input);
  cond_rem.filter(*out);

}

void CloudViewer::processCloud(const VPointCloud::ConstPtr &scan)
{
  ROS_DEBUG_STREAM(" process cloud "<<scan->header.stamp);
  ros::WallTime wall_start = ros::WallTime::now();
  ros::Time start = ros::Time::now();
  double latency = (start - ros::Time().fromNSec(scan->header.stamp * 1e3)).toSec();
  ROS_DEBUG_STREAM(" latency = "<<latency);
  ros::Time scan_time = pcl_conversions::fromPCL(scan->header).stamp;
  tf::StampedTransform scan_transform;
  try
  {
    if (!tfListener.waitForTransform(map_frame, scan->header.frame_id, scan_time, ros::Duration(0.2)))
    {
      ROS_WARN_STREAM("no transform from "<<scan->header.frame_id<<" to "<<map_frame<< " for time "<<scan_time);
      return;
    }
    tfListener.lookupTransform(map_frame, scan->header.frame_id, scan_time, scan_transform);

  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  double wait_transform_time = (ros::Time::now() - start).toSec();
  ROS_DEBUG_STREAM_COND(wait_transform_time > 0.03, "wait transform time = "<<wait_transform_time);

  VPointCloud::Ptr map_scan = boost::make_shared<VPointCloud>();
  if (scan->header.frame_id != map_frame)
  {
    VPointCloud::Ptr fltr_scan = boost::make_shared<VPointCloud>();
    filter(scan, fltr_scan);
    pcl_ros::transformPointCloud(*fltr_scan, *map_scan, scan_transform);

  }
  else
  {
    //just copy scan
    *map_scan = *scan;

  }
  block lock(mutex);
  cloud_que.push(map_scan);
  ROS_DEBUG_STREAM("process wall time = "<<(ros::WallTime::now() - wall_start).toSec());
}

void CloudViewer::run()
{
  ROS_DEBUG_STREAM("run "<<tms);
  boost::thread thread(&CloudViewer::viewer_thread, this);
  while (ros::ok() && !stopped)
  {
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(tms));
  }
  stopped = true;
}

void CloudViewer::viewer_thread()
{
  while (!viewer->wasStopped() && !stopped)
  {
    // boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    {
      block lock(mutex);
      while (!cloud_que.empty())
      {
        //addIRGBCloud(cloud_que.front(), r, g, b);
        addHRGBCloud(cloud_que.front(), min_z, max_z);
        cloud_que.pop();
      }
    }
    ros::WallTime wall_begin = ros::WallTime::now();
    viewer->spinOnce(tms/2);
    boost::this_thread::sleep(boost::posix_time::milliseconds(tms/2));
    //ROS_DEBUG_STREAM("view time = "<<(ros::WallTime::now() - wall_begin).toSec() );
  }
  stopped = true;
}

CloudViewer::CloudViewer(ros::NodeHandle& nh) :
    Viewer("Cloud viewer"),
    map_frame(nh.param<std::string>("map_frame", "map")),
    tms(nh.param("tms", 50000)),
    r(nh.param("r", 1.0)),
    g(nh.param("g", 0.0)),
    b(nh.param("b", 0.0)),
    grid_size(nh.param("grid_size", grid_size)),
    rad(nh.param("rad", 1.0)),
    max_rad(nh.param("max_rad", 20.0)),
    min_rad(nh.param("min_rad", 5.0)),
    min_z(nh.param("min_z", -2.0)),
    max_z(nh.param("max_z", 0.0)),
    laser_sub(nh.subscribe("/points", 1, &CloudViewer::processCloud, this, ros::TransportHints().tcpNoDelay(true))),
    stopped(false)
{
  build_range_filter();
  build_grid_filter();
}

CloudViewer::~CloudViewer()
{

}

