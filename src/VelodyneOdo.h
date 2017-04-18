/*
 * VelodyneOdo.h
 *
 *  Created on: 28.04.2016
 *      Author: Andrey
 */

#ifndef VELODYNEODO_H_
#define VELODYNEODO_H_

#include "PCRegistration.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "vcloud.h"
#include <boost/shared_ptr.hpp>
#include "CloudNormalizer.h"
#include "CloudSparser.h"

namespace registration
{

class VelodyneOdo
{
protected:
  boost::shared_ptr<PCRegistration> RegPtr;
  boost::shared_ptr<CloudNormalizer> pc_normalizer;
  //sparser for first cloud
  boost::shared_ptr<CloudSparser> first_sparser;
  //sparser for next cloud
  boost::shared_ptr<CloudSparser> next_sparser;

  //current velocities
  tf::Vector3 linear_velocity;
  tf::Vector3 angular_velocity;

  ros::Subscriber velodyneSub;
  ros::Subscriber odoSub;
  ros::Subscriber insSub;

  ros::Publisher pcPub;
  ros::Publisher lodoPub;
  std::string odo_frame;
  std::string laser_frame;
  //true if need to transform result pointcloud into odo frame
  bool transfrom_pointcloud;
  tf::TransformBroadcaster tf;
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;

  int rings_number;
  //different criterion s for restarting registration process
  //distance criterion
  double registration_distance;
  double max_registration_distance;
  //time criterion
  double registration_time;
  long int max_registration_time;
  //number of alignment iterations
  int max_alignment_iterations;
  //last cloud after normalization
  VPointCloud::Ptr normalized_cloud;
  //last processed cloud header
  pcl::PCLHeader last_cloud_header;
  //pointer to first cloud inserted into registration map
  VPointCloud::Ptr first_cloud;

  //transformation wrt start pose
  QTransform transform;
  //transformation wrt last registration map map
  QTransform local_transform;

  //step transform
  QTransform delta_transform;

  //first cloud transform
  QTransform first_transform;

  //frame id for tf
  std::string base_frame;
  //laser pose wrt to base_link
  tf::StampedTransform laser_transform;

  //minimal percent of active points in alignment
  double active_point_percent;
  //minimal part of good points in normalized scan
  double min_point_part;
  void init();
public:
  void onOdo(const nav_msgs::OdometryConstPtr& odo){}
  void onImu(const sensor_msgs::ImuConstPtr& imu){}

  void detectRings(const VPointCloud::ConstPtr &scan);
  void processCloud(const VPointCloud::ConstPtr &scan);

  void publishResults();
  void publishOdoTF();
  void publishPC();

  //Registration function returns true if scan is registered and all registration conditions fullfilled
  bool findTransform(double dt);

  VelodyneOdo(ros::NodeHandle& _nh, ros::NodeHandle& _priv_nh);

};

} /* namespace registration */
#endif /* VELODYNEODO_H_ */
