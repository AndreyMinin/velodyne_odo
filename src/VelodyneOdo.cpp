/*
 * VelodyneOdo.cpp
 *
 *  Created on: 28.04.2016
 *      Author: user
 */

#include "VelodyneOdo.h"
#include <tf/transform_listener.h>

namespace registration
{

void VelodyneOdo::detectRings(const VPointCloud::ConstPtr &scan)
{
  ROS_DEBUG_STREAM("VelodyneOdo: detectRings "<<scan->header.seq);
  rings_number = 0;

  for( VPointCloud::const_iterator it = scan->begin();
        it != scan->end(); it++)
  {

    if ( it->ring > rings_number)
          rings_number = it->ring;

    //ROS_INFO_STREAM("index = "<<i<<" ring = "<<it->ring<<" angle = "<<atan2(it->y, it->x));

  }
  rings_number += 1;

  ROS_DEBUG_STREAM("rings number detected = "<<rings_number);

  velodyneSub.shutdown();
  //normalized_cloud->header = scan->header;
  last_cloud_header = scan->header;
  laser_frame = scan->header.frame_id;
  init();
}


void VelodyneOdo::publishPC()
{
  if ( !transfrom_pointcloud )
  {
    pcPub.publish(normalized_cloud);
  }
  else
  {
    static VPointCloud cloud;
    cloud.header = normalized_cloud->header;
    cloud.header.frame_id = odo_frame;
    tf::Transform odo_transform(laser_transform);
    odo_transform*=transform.transform();
    cloud.reserve(normalized_cloud->size());
    for( size_t i = 0; i< normalized_cloud->size(); i++)
    {
      VPoint& p = normalized_cloud->operator[](i);
      if ( p.x*p.x + p.y*p.y + p.z*p.z > 0.01 )
      {
        tf::Vector3 v(p.x, p.y, p.z);
        v = odo_transform(v);
        VPoint odo_p;
        odo_p.intensity = p.intensity;
        odo_p.ring = p.ring;
        odo_p.x = v.x(); odo_p.y = v.y(); odo_p.z = v.z();
        cloud.push_back(odo_p);
      }
    }
    pcPub.publish(cloud);
    cloud.clear();
  }
}

void VelodyneOdo::publishOdoTF()
{
  tf::Transform odo_transform(laser_transform);
  odo_transform*=transform.transform();
  odo_transform*=laser_transform.inverse();
  tf::StampedTransform STrans(odo_transform,
                         ros::Time().fromNSec(normalized_cloud->header.stamp*1000),
                           odo_frame,
                           base_frame);
  tf.sendTransform(STrans);

  nav_msgs::Odometry odo_msg;
  odo_msg.header.seq = normalized_cloud->header.seq;
  odo_msg.header.frame_id = odo_frame;
  odo_msg.header.stamp.fromNSec(normalized_cloud->header.stamp * 1000);
  odo_msg.pose.pose.position.x = STrans.getOrigin().x();
  odo_msg.pose.pose.position.y = STrans.getOrigin().y();
  odo_msg.pose.pose.position.z = STrans.getOrigin().z();
  tf::quaternionTFToMsg(STrans.getRotation(), odo_msg.pose.pose.orientation);

  odo_msg.child_frame_id = base_frame;
  //ratation matrix for laser
  tf::Matrix3x3 laser_matrix = laser_transform.getBasis();
  tf::vector3TFToMsg(laser_matrix*linear_velocity,
                     odo_msg.twist.twist.linear);
  tf::vector3TFToMsg(laser_matrix*angular_velocity,
                     odo_msg.twist.twist.angular);
  lodoPub.publish(odo_msg);
}

void VelodyneOdo::publishResults()
{
  publishPC();
  publishOdoTF();
}

std::ostream& operator << (std::ostream& stream, const tf::Matrix3x3& m)
{
  stream<<m[0][0]<<" "<<m[0][1]<<" "<<m[0][2]<<std::endl
      <<m[1][0]<<" "<<m[1][1]<<" "<<m[1][2]<<std::endl
      <<m[2][0]<<" "<<m[2][1]<<" "<<m[2][2]<<std::endl;
  return stream;
}

std::ostream& operator << (std::ostream& stream, const tf::Vector3& v)
{
  stream<<v[0]<<" "<<v[1]<<" "<<v[2];
  return stream;
}

void print_transform(const tf::Transform& transf, const std::string& msg="")
{
  //transf.getOrigin().setZ(0);
  tf::Vector3 V = transf.getOrigin();
  ROS_INFO_STREAM(msg<<" translation = "<<V<<" modul = "<<V.length());
  tf::Quaternion Q = transf.getRotation();
  tf::Vector3 axe = Q.getAxis();
  ROS_INFO_STREAM(msg<<"rotation = "<<axe<<" angle = "<<Q.getAngle());
  ROS_INFO_STREAM_COND(axe.length()<0.99, " bad matrix = "<<transf.getBasis());
  ROS_ASSERT( fabs(Q.length() -1 ) < 0.001 );
}


bool VelodyneOdo::findTransform( double dt)
{
  QTransform old_trans(local_transform);

  local_transform *= delta_transform;
  //sparsing
  VPointCloud::Ptr sparse_cloud =
      next_sparser->sparse(normalized_cloud/*, local_transform*/);
  ROS_DEBUG_STREAM("sparse count = "<<sparse_cloud->size());

  int iterations = RegPtr->findTransform(sparse_cloud, local_transform);
  if ( iterations <= 0 )
  {
    ROS_WARN_STREAM("failed to find transform "<<iterations);
  }
//  delta = old (-1) * T
  delta_transform.mult(old_trans.inverse(), local_transform);
  //increment global transform
  transform = first_transform * local_transform;//*= delta_transform;

  linear_velocity = delta_transform.getOrigin() /dt;
  angular_velocity = delta_transform.getRotation().getAxis()*
      ( delta_transform.getRotation().getAngle() / dt );
  ROS_DEBUG_STREAM("v = "<<linear_velocity);
  ROS_DEBUG_STREAM("w = "<<angular_velocity);
  double local_distance = local_transform.getOrigin().length();
  ROS_DEBUG_STREAM("distance = "<<local_distance);
  publishResults();

  //check registration restart conditions
  if( iterations < 0 || iterations > max_alignment_iterations )
  {
    ROS_DEBUG_STREAM("VelodyneOdo: iterations number exceeded");
    return false;
  }

  if (RegPtr->activePointsQuantity() < active_point_percent * sparse_cloud->size() )
  {
    ROS_DEBUG_STREAM("VelodyneOdo: active points number under limit");
    return false;
  }

  if (local_distance > max_registration_distance)
  {
    ROS_DEBUG_STREAM("VelodyneOdo: registration distance exceeded");
    return false;
  }
  if (normalized_cloud->header.stamp - first_cloud->header.stamp > max_registration_time)
  {
    ROS_DEBUG_STREAM("VelodyneOdo: registration lifetime exceeded");
    return false;
  }
  //all else - continue registration
  return true;
}

void VelodyneOdo::processCloud(const VPointCloud::ConstPtr &scan)
{
  ROS_DEBUG_STREAM(std::endl<<"VelodyneOdo: process cloud "<<scan->header.seq<<
          " latency = "<<
          (ros::Time::now() - ros::Time().fromNSec(scan->header.stamp*1e3)).toSec());

  //time from last scan
  double dt = ( scan->header.stamp - last_cloud_header.stamp )*1.0e-6;

  int increment = scan->header.seq - last_cloud_header.seq;
  ROS_WARN_STREAM_COND(increment>1, "missed a scan "<<increment);
  //scan acquisition time
  double cloud_dt =  dt / increment;

  ROS_DEBUG_STREAM("dt = "<<dt<<" cloud_dt = "<<cloud_dt);
  ROS_DEBUG_STREAM("initial point number = "<<scan->size());
  size_t normalize_size = pc_normalizer->normalize(normalized_cloud, scan,
                           linear_velocity, angular_velocity, cloud_dt);
  ROS_DEBUG_STREAM("normalized count "<<normalize_size);
  if ( normalize_size <= normalized_cloud->size()*min_point_part  )
  {
    ROS_WARN_STREAM(" bad cloud normalized size = "<<normalize_size);
    return;
  }
  last_cloud_header = scan->header;
  //if registration map not empty ( we have first cloud )
  if ( first_cloud )
  {
    //if registration successful
    if ( findTransform( dt))
      return;
  }
  print_transform(transform.transform(), "transform");
  //both else  - insert new scan into registration map
  if ( first_cloud ) //registration map not empty - we should clean it
  {
    RegPtr->clearPointCloudFast(first_cloud);
    ROS_DEBUG_STREAM("map cleared");
  }

  //insert last cloud into registration map
  first_cloud = first_sparser->sparse(normalized_cloud);
  ROS_DEBUG_STREAM("sparse count = "<<first_cloud->size());
  RegPtr->setPointCloudFast(first_cloud);
  ROS_DEBUG_STREAM("new cloud inserted");

  local_transform.setIdentity();
  first_transform = transform;

}


template<class T>
T loadParam(ros::NodeHandle& node, const std::string& name, const T& def)
{
  T val(def);
  node.param(name, val, def);
  return val;
}

void VelodyneOdo::init()
{
  double Xmax = loadParam(priv_nh, "Xmax", 40.0);
  double Ymax = loadParam(priv_nh, "Ymax", 40.0);
  double Zmax = loadParam(priv_nh, "Zmax", 2.0);
  double Zmin = loadParam(priv_nh, "Zmin", -3.0);
  double minRad = loadParam(priv_nh, "minRad", 5.0);
  double maxRad = std::min(Xmax, Ymax);
  double sparse_first = loadParam(priv_nh, "sparse_first", 0.5);
  double sparse_next = loadParam(priv_nh, "sparse_next", 0.5);
  double cell_size = loadParam(priv_nh, "cell_size", 0.2);
  double sph_rad = loadParam(priv_nh, "sph_size", 1.0);
  max_registration_distance = loadParam(priv_nh, "registration_distance", 8.0);
  max_registration_time = loadParam(priv_nh, "registration_time", 30.0) * 1000000;
  double max_dis = loadParam(priv_nh, "max_dis", 0.005);
  double max_rot = loadParam(priv_nh, "max_rot", 0.001);
  double min_dis = loadParam(priv_nh, "min_dis", 0.0001);
  double min_rot = loadParam(priv_nh, "min_rot", 0.0001);
  int max_iterations = loadParam(priv_nh, "max_iterations", 200);

  max_alignment_iterations = loadParam(priv_nh, "max_alignment_iterations", 60);
  active_point_percent = loadParam(priv_nh, "active_point_percent", 0.6);
  min_point_part = loadParam(priv_nh, "min_point_part", 0.5);
  int rays = loadParam(priv_nh, "rays", 360);

  odo_frame = loadParam<std::string>(priv_nh, "odo_frame", "odom");
  transfrom_pointcloud = loadParam(priv_nh, "transform_pointcloud", false);

  base_frame = loadParam<std::string>(priv_nh, "base_frame", "base_link");
  pcPub = priv_nh.advertise<sensor_msgs::PointCloud2>("normalized_cloud", 10);
  lodoPub = priv_nh.advertise<nav_msgs::Odometry>("odo", 10);
  float mem_req = 1e-9 * (float)(2*Xmax/cell_size*2*Ymax/cell_size*(Zmax-Zmin)/cell_size*sizeof(cell_t));
  ROS_INFO_STREAM("memory requirement "<<mem_req<< " Gbyte");
  int max_gb = loadParam(priv_nh, "max_gb", 2);
  ROS_ASSERT(mem_req < max_gb);

  std::vector<double> start_mask_angles;
  std::vector<double> finish_mask_angles;
  priv_nh.getParam("start_mask_angles", start_mask_angles);
  priv_nh.getParam("finish_mask_angles", finish_mask_angles);

  first_sparser = boost::make_shared<CloudSparser>(sparse_first,
                            - Xmax, - Ymax,  Zmin,
                            2 * Xmax, 2 * Ymax, Zmax - Zmin, maxRad, minRad);

  next_sparser = boost::make_shared<CloudSparser>(sparse_next,
                              - Xmax, - Ymax,  Zmin,
                              2 * Xmax, 2 * Ymax, Zmax - Zmin, maxRad, minRad);

  pc_normalizer = boost::make_shared<CloudNormalizer>(rings_number, rays,
                                                      start_mask_angles,
                                                      finish_mask_angles);

  double delta_= 2*sph_rad;
  RegPtr.reset( new PCRegistration(
                        - Xmax-delta_, - Ymax-delta_,  Zmin -delta_,
                         2*Xmax + 2*delta_ , 2*Ymax + 2*delta_, Zmax - Zmin+2*delta_,
                         cell_size, sph_rad,
                         max_iterations,
                         max_dis,    max_rot,
                         min_dis ,    min_rot ) );

  linear_velocity.setZero();
  angular_velocity.setZero();
  transform.setIdentity();
  local_transform.setIdentity();
  delta_transform.setIdentity();


  ROS_DEBUG_STREAM("wait for laser to base transform");

  tf::TransformListener tfListener;
  if ( !tfListener.waitForTransform(
      base_frame,
      laser_frame,
      ros::Time(0), ros::Duration(10.0)) )
  {
    ROS_ERROR_STREAM("can not find transform from "
                    <<laser_frame
                     <<" to "<<base_frame);
    return;
  }

  tfListener.lookupTransform(base_frame, laser_frame, ros::Time(0), laser_transform);
  velodyneSub = nh.subscribe("point_cloud", 10, &VelodyneOdo::processCloud, this,
                             ros::TransportHints().tcpNoDelay(true));
  ROS_INFO_STREAM("VelodyneOdo: start processing");
}

VelodyneOdo::VelodyneOdo(ros::NodeHandle& _nh, ros::NodeHandle& _priv_nh):
  nh(_nh), priv_nh(_priv_nh),
  registration_distance(0.0),
  registration_time(0.0),
  normalized_cloud(new VPointCloud)
{
  velodyneSub = nh.subscribe("point_cloud", 10, &VelodyneOdo::detectRings, this,
                             ros::TransportHints().tcpNoDelay(true));
}

} /* namespace registration */
