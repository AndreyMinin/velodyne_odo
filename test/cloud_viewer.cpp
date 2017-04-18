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
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>

void CloudViewer::simple_filter_pcloud(const VPointCloud::ConstPtr& input,
                                VPointCloud::Ptr& out)
{
    out->reserve(input->size());
    double max2 = max_rad*max_rad;
    for( int i = 0; i< input->size(); ++i)
    {
        const VPoint& p = input->points[i];

        if ( p.x * p.x + p.y*p.y + p.z*p.z > max2)
        {
            continue;
        }
        out->push_back(p);
    }
}

void CloudViewer::filter_pcloud(const VPointCloud::ConstPtr& input,
                                VPointCloud::Ptr& output)
{
    ros::WallTime wall_start = ros::WallTime::now();
    pcl::PassThrough<VPoint> pass(true);
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-max_rad, max_rad);
    boost::shared_ptr< std::vector<int> > indexes_x_ptr = boost::make_shared< std::vector<int> >();
    pass.filter(*indexes_x_ptr);
    pass.setIndices(indexes_x_ptr);
    ROS_DEBUG_STREAM(" x size "<<indexes_x_ptr->size());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-max_rad, max_rad);
    boost::shared_ptr< std::vector<int> > indexes_xy_ptr = boost::make_shared< std::vector<int> >();
    pass.filter(*indexes_xy_ptr);
    pass.setIndices(indexes_xy_ptr);
    ROS_DEBUG_STREAM(" xy size "<<indexes_xy_ptr->size());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-max_rad, max_rad);


    if ( grid_size < 0 )
    {
        pass.filter(*output);
    }
    else
    {
        boost::shared_ptr< std::vector<int> > indexes_xyz_ptr = boost::make_shared< std::vector<int> >();
        pass.filter(*indexes_xyz_ptr);
        ROS_DEBUG_STREAM(" xyz size "<<indexes_xyz_ptr->size());
        pcl::VoxelGrid< VPoint > grid;
        grid.setInputCloud(input);
        grid.setIndices(indexes_xyz_ptr);
        grid.setLeafSize(grid_size, grid_size, grid_size);
        grid.filter(*output);
    }
    //ROS_DEBUG_STREAM("filter wall time = "<<(ros::WallTime::now() - wall_start).toSec() );
    ROS_DEBUG_STREAM("input size = "<<input->size()<<" filtered size = "<<output->size());
}


void CloudViewer::processCloud(const VPointCloud::ConstPtr &scan)
{
    ROS_DEBUG_STREAM(" process cloud "<<scan->header.stamp);
    ros::WallTime wall_start = ros::WallTime::now();
    ros::Time start = ros::Time::now();
    double latency = (start - ros::Time().fromNSec(scan->header.stamp*1e3)).toSec();
    ROS_DEBUG_STREAM(" latency = "<<latency);
    ros::Time scan_time = pcl_conversions::fromPCL(scan->header).stamp;
    tf::StampedTransform scan_transform;
    try
    {
        if ( ! tfListener.waitForTransform(map_frame,
                                    scan->header.frame_id,
                                    scan_time, ros::Duration(0.2) ) )
        {
          ROS_WARN_STREAM("no transform from "<<scan->header.frame_id<<" to "<<map_frame<<
                          " for time "<<scan_time);
          return ;
        }
        tfListener.lookupTransform(map_frame, scan->header.frame_id, scan_time,
                            scan_transform);

    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("%s",ex.what());
      return;
    }
    double wait_transform_time = (ros::Time::now() - start).toSec();
    ROS_DEBUG_STREAM_COND(wait_transform_time > 0.03, "wait transform time = "<<wait_transform_time);

    VPointCloud::Ptr map_scan = boost::make_shared<VPointCloud>();
    if ( scan->header.frame_id != map_frame )
    {
        VPointCloud::Ptr fltr_scan = boost::make_shared<VPointCloud>();
        filter_pcloud(scan, fltr_scan);

        pcl_ros::transformPointCloud( *fltr_scan, *map_scan, scan_transform );

    }
    else
    {
      //just copy scan
        *map_scan = *scan;

    }
    block lock(mutex);
    cloud_que.push(map_scan);
    ROS_DEBUG_STREAM("process wall time = "<<(ros::WallTime::now() - wall_start).toSec() );
}

void CloudViewer::run()
{
  ROS_DEBUG_STREAM("run "<<tms);
  boost::thread thread(&CloudViewer::viewer_thread, this);
  while( ros::ok() && !stopped )
  {
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(tms));
  }
  stopped = true;
}

void CloudViewer:: viewer_thread()
{
  while ( !viewer->wasStopped() && !stopped )
  {
    // boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    {
      block lock(mutex);
      while( !cloud_que.empty())
      {
        //addIRGBCloud(cloud_que.front(), r, g, b);
        addHRGBCloud(cloud_que.front(), min_z, max_z);
        cloud_que.pop();
      }
    }
    ros::WallTime wall_begin = ros::WallTime::now();
    viewer->spinOnce(tms);
    //ROS_DEBUG_STREAM("view time = "<<(ros::WallTime::now() - wall_begin).toSec() );
  }
  stopped = true;
}

CloudViewer::CloudViewer(ros::NodeHandle& nh):
  Viewer("Cloud viewer"),
  map_frame(nh.param<std::string>("map_frame", "map")),
  tms(nh.param("tms", 50000)),
  r(nh.param("r", 1.0)),
  g(nh.param("g", 0.0)),
  b(nh.param("b", 0.0)),
  grid_size(nh.param("grid_size", grid_size)),
  rad(nh.param("rad", 1.0)),
  max_rad(nh.param("max_rad", 20.0)),
  min_z(nh.param("min_z", -2.0)),
  max_z(nh.param("max_z", 0.0)),
  laser_sub(nh.subscribe("/points", 10, &CloudViewer::processCloud, this,
                         ros::TransportHints().tcpNoDelay(true))),
  stopped(false)
{

}

CloudViewer::~CloudViewer()
{

}

