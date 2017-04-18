/*
 * VelodyneOdoMT.h
 *
 *  Created on: 28.04.2016
 *      Author: Andrey
 */

#ifndef VELODYNEODOMT_H_
#define VELODYNEODOMT_H_

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

class RegistrationAttr
{
public:
  enum eState
  {
    busy_state, //map is busy by second thread (while cleaning or inserting first scan)
    ready_state, //map is ready to registration (first scan is inserted)
    clean_state  //map is clean and ready for first scan
  };
  eState state;
  //count of clouds registered after restart condition
  int over_count;
  boost::shared_ptr<PCRegistration> RegPtr;
  //transformation wrt current registration map pose
  QTransform local_transform;
  //pointer to first cloud inserted into registration map
  VPointCloud::Ptr first_cloud;
  //transform of first scan
  QTransform first_transform;
  //sparser for first cloud
  boost::shared_ptr<CloudSparser> first_sparcer;
  //clean map
  void clean();
  //insert first cloud into registration map
  void setFirst(const VPointCloud::Ptr& cloud);
  RegistrationAttr(): state(ready_state), over_count(-1)
  {
    local_transform.setIdentity();
    first_transform.setIdentity();
  }
};

class VelodyneOdoMT
{
protected:
  RegistrationAttr RegAttr[2];

  RegistrationAttr* active;
  RegistrationAttr* inactive;
  boost::shared_ptr<CloudNormalizer> pc_normalizer;

  //sparser for next cloud
  boost::shared_ptr<CloudSparser> next_sparcer;

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


  //transformation wrt start pose
  QTransform transform;
  //step transform
  QTransform delta_trans;
  //frame id for tf
  std::string base_frame;
  //laser pose wrt to base_link
  tf::StampedTransform laser_transform;
  //max number of cloud to process before switch to inactive registration
  int max_overcount;
  //minimal percent of active points in alignment
  double active_point_percent;
  //minimal part of good points in normalized scan
  double min_point_part;
  void init();
  //starts thread to clean inactive map
  void startCleanInactive();
  //starts thread inserting point cloud into inactive map
  void setCloudInactive(const VPointCloud::Ptr& ptr);
public:
  void onOdo(const nav_msgs::OdometryConstPtr& odo){}
  void onImu(const sensor_msgs::ImuConstPtr& imu){}

  void detectRings(const VPointCloud::ConstPtr &scan);
  void initMap(const VPointCloud::ConstPtr &scan);
  void processCloud(const VPointCloud::ConstPtr &scan);

  void publishResults();
  void publishOdoTF();
  void publishPC();

  //Registration function
  bool findTransform(double dt);

  VelodyneOdoMT(ros::NodeHandle& _nh, ros::NodeHandle& _priv_nh);

};

} /* namespace registration */
#endif /* VELODYNEODO_H_ */
