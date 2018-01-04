/*
 * SphereArr.cpp
 *
 *  Created on: 13.11.2014
 *      Author: user
 */

#include "SphereArr.h"
#include <cmath>
#include <iostream>
#include <cassert>
#include "Map3D.h"
#include "PCRegistration.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Transform.h"
#include <boost/bind.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <velodyne_pointcloud/rawdata.h>
#ifdef SHOW_VIEWER
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#endif
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

#include "ATan.h"
#include <fstream>
#include "vcloud.h"
#include "CloudSparser.h"
#include "CloudNormalizer.h"
//#define SHOW_VIEWER


//namespace registration {
//
//size_t normalize_cloud(const VPointCloud::ConstPtr& in,
//                     const VPointCloud::Ptr& out,
//                     const ATan& Atan,
//                     size_t ring_number);
//
//void correct_cloud(const VPointCloud::Ptr& normalized_cloud,
//                   size_t ring_number,
//                   const tf::Vector3& linear_velocity,
//                   const tf::Vector3& angular_velocity,
//                   double dt,
//                   size_t first_ray_index = 0);
//
//}  // namespace registration


using namespace registration;
using std::fabs;
using tf::Vector3;
using pcl::transformPointCloud;

template<class T>
class Modul1
{
public :
  T operator ()(int i, int j, int k, T t)
  {
    return std::fabs(t);
  }
};

template<class T>
class Quadric
{
public :
  T operator ()(int i, int j, int k, T t)
  {
    return t*t;
  }
};


bool  stop = false;
bool quit = false;
bool vpause = false;
bool vstep = false;


#ifdef SHOW_VIEWER
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> viewPtr;

viewPtr simpleVis ()
{
  // --------------------------------------------
  // -----Open 3D viewer -----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
   viewer->initCameraParameters ();
  return (viewer);
}

static int cloud_count(0);
static std::string cloud_name;


void addCloud(const viewPtr& viewer,
              VPointCloud::ConstPtr cloud, double rad =1.0 ,
              double r = 1.0 , double g = 1.0,  double b = 1.0 )
{
  cloud_count++;
  std::stringstream ss;
  ss<<"cloud_"<<cloud_count;
//  ROS_INFO_STREAM("add cloud "<<ss.str());
  cloud_name = (ss.str());
  viewer->addPointCloud<VPoint>(cloud, cloud_name);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                              rad, cloud_name);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                r, g, b, cloud_name);
}

void addRGBCloud(const viewPtr& viewer,
              RGBCloud::ConstPtr cloud, double rad =1.0 )
{
  cloud_count++;
  std::stringstream ss;
  ss<<"cloud_"<<cloud_count;
//  ROS_INFO_STREAM("add cloud "<<ss.str());
  cloud_name = (ss.str());
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_name);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                              rad, cloud_name);

}



void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  viewPtr viewer = *static_cast<viewPtr *> (viewer_void);
  if ( event.keyDown ())
  {
    std::cout << "key was pressed " << std::endl;

    switch (event.getKeyCode()) {
      case 'q':
      case 'x':
        std::cout<<" quit "<<std::endl;
        quit = true;
        break;
      case 'p':
        std::cout<<" pause "<<std::endl;
        vpause = !vpause;
        break;
      case 's':
        std::cout<<" step "<<std::endl;
        vstep  = !vstep;
        break;
      default:
        break;
    }


    stop = true;
  }
}

void showVis(viewPtr& viewer)
{

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

void showVis(viewPtr& viewer, int tms)
{
  if ( vstep )
    vpause = true;
  boost::signals2::connection conn;

  conn = viewer->registerKeyboardCallback (keyboardEventOccurred,
                                      (void*)&viewer);
  while ( (!viewer->wasStopped() && tms != 0 )
      || vpause)
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    if ( tms <= 0 )
    {
      if ( stop )
        break;
    }
    else
    {
      tms -= 100;
    }

  }
  stop = false;
  if ( quit )
  {
    viewer->close();
    //exit(0);
  }

  conn.disconnect();
}
#endif

template <class F>
float measure_time(const F& f)
{
  ros::WallTime start = ros::WallTime::now();
  f();
  ros::WallTime end = ros::WallTime::now();
  return ros::WallDuration((end - start)).toSec();
}

template <class F, class R >
float measure_time(const F& f, R& r)
{
  ros::WallTime start = ros::WallTime::now();
  r = f();
  ros::WallTime end = ros::WallTime::now();
  return ros::WallDuration((end - start)).toSec();
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

  tf::Vector3 V = transf.getOrigin();
  ROS_INFO_STREAM(msg<<" translation = "<<V<<" modul = "<<V.length());
 // ROS_INFO_STREAM("rotation matrix "<<transf.getBasis());
  tf::Quaternion Q = transf.getRotation();
  tf::Vector3 axe = Q.getAxis();
  ROS_INFO_STREAM(msg<<" rotation = "<<axe<<" angle = "<<Q.getAngle());
  ROS_INFO_STREAM_COND(axe.length()<0.99, " bad matrix = "<<transf.getBasis());
  ROS_ASSERT( fabs(Q.length() -1 ) < 0.001 );
}



#ifdef SHOW_VIEWER
void addSparceCloud2(const viewPtr& viewer, const VPointCloud::Ptr& cloud,
                    const int miss,
                    const double Xmin, const double Ymin, const double Zmin,
                    const double Xmax, const double Ymax, const double Zmax,
                    int size, double r,  double g, double b)
{
  //VPointCloud::Ptr cloud_res(new VPointCloud);
  RGBCloud::Ptr cloud_res(new RGBCloud);
  cloud_res->reserve(cloud->size() /( miss+1));
  for (int i = 0; i < cloud->size(); i += (miss+1))
  {
    VPoint& p = (*cloud)[i];
    //      if ( p.ring != 1 && p.ring != 2)
    //        continue;
    if (p.x > Xmax)
      continue;
    else if (p.x < Xmin)
      continue;

    if (p.y > Ymax)
      continue;
    else if (p.y < Ymin)
      continue;

    if (p.z > Zmax)
      continue;
    else if (p.z < Zmin)
      continue;

    pcl::PointXYZRGB out_p;
    out_p.x = p.x;
    out_p.y = p.y;
    out_p.z = p.z;
    float intensity = 250; //p.intensity;
    out_p.r = r * intensity;
    out_p.g = g * intensity;
    out_p.b = b * intensity;
    cloud_res->push_back(out_p);
    //cloud_res->push_back(p);
  }
  //addCloud(viewer, cloud_res, size, r, g, b);
  addRGBCloud(viewer, cloud_res, size);
}

//void changeLastCloud(const viewPtr& viewer, const VPointCloud::Ptr& cloud,
//                     const int miss, const double Xmax, const double Ymax, const double Zmax,
//                     int size, double r,  double g, double b)
//{
//  viewer->removePointCloud(cloud_name);
//  addSparceCloud(viewer, cloud, miss, Xmax, Ymax, Zmax, r, r, g, b);
//}

void changeLastCloud2(const viewPtr& viewer, const VPointCloud::Ptr& cloud,
                     const int miss,
                     const double Xmin, const double Ymin, const double Zmin,
                     const double Xmax, const double Ymax, const double Zmax,
                     int size, double r,  double g, double b)
{
  if ( !cloud_name.empty() )
  {
//    ROS_INFO_STREAM("remove cloud "<<cloud_name);
    viewer->removePointCloud(cloud_name);
  }
  addSparceCloud2(viewer, cloud, miss,Xmin, Ymin, Zmin, Xmax, Ymax, Zmax, size, r, g, b);
}

#endif



static ros::Publisher map_pub;
static ros::Publisher twist_pub;

void publish_twist(QTransform& vel)
{
  geometry_msgs::Twist twist;
  tf::vector3TFToMsg(vel.getOrigin(), twist.linear);
  tf::vector3TFToMsg(
      vel.getRotation().getAxis() * vel.getRotation().getAngle() ,
      twist.angular);
  twist_pub.publish(twist);
}

void publish_map(PCRegistration& Reg, double z)
{
  static int seq(0);
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "map";
  map.header.seq = seq++;
  map.header.stamp = ros::Time::now();
  map.info.height = Reg.getMap().cellSizeY();
  map.info.width = Reg.getMap().cellSizeX();
  map.info.resolution = Reg.getMap().resolution();
  map.info.origin.orientation.w = 1.0;
  map.info.origin.position.x = Reg.getMap().origin_x();
  map.info.origin.position.y = Reg.getMap().origin_y();
  map.info.origin.position.z = Reg.getMap().origin_z();
  Reg.getHorisontalSlice(map.data, z);
  ROS_INFO_STREAM("publish map");
  map_pub.publish(map);
}

template<class T>
T loadParam(ros::NodeHandle& node, const std::string& name, const T& def)
{
  T val(def);
  node.param(name, val, def);
  return val;
}

bool is_cell_notzero(cell_t& cell)
{
  bool res = (cell.range != EMPTY_CELL) ||
      (cell.x != 0) || (cell.y != 0) || (cell.z != 0);
  ROS_INFO_STREAM_COND(res, " non zero cell "<<cell.range<<" "<<cell.x<<" "<<cell.y<<" "<<cell.z);
  return res;
}


tf::Vector3 get_average(const std::list<tf::Vector3>& list)
{
  if ( list.empty() )
      return tf::Vector3(0,0,0);

  tf::Vector3 avg(0.0, 0.0, 0.0);
  int n = 0;
  for(typename std::list<tf::Vector3>::const_iterator it = list.begin();
      it != list.end(); it++)
  {
    avg += *it;
    n++;
  }
  return avg / n;
}

void load_from_bag_sparse_full()
{
#ifdef SHOW_VIEWER
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = simpleVis();
#endif

  ros::NodeHandle ng;
  map_pub = ng.advertise<nav_msgs::OccupancyGrid>("map", 10);
  twist_pub = ng.advertise<geometry_msgs::Twist>("twist", 10);

  ros::NodeHandle node("~");
  std::string bag_name = loadParam<std::string>(node, "bag_name", "");
  if ( bag_name.empty() )
  {
    ROS_ERROR_STREAM("no bag file specified ");
    return;
  }
#ifdef SHOW_VIEWER
  vpause = loadParam(node, "pause", false);
#endif
  double key_tms = loadParam(node, "key_tms", 4000.0);
  double cadr_tms = loadParam(node, "cadr_tms", 4000.0);
  double cadr_result_tms = loadParam(node, "cadr_result_tms", 4000.0);

  bool wait_input = loadParam(node, "wait_input", false);
  double Xmax = loadParam(node, "Xmax", 40.0);
  double Ymax = loadParam(node, "Ymax", 40.0);
  double Zmax = loadParam(node, "Zmax", 2.0);
  double Zmin = loadParam(node, "Zmin", -3.0);
  double minRad = loadParam(node, "minRad", 5.0);
  double maxRad = std::min(Xmax, Ymax);
  int miss_set = loadParam(node,"miss_set", 0);
  int miss_check = loadParam(node,"miss_check", 0);
  double sparse_first = loadParam(node, "sparse_first", 0.5);
  double sparse_next = loadParam(node, "sparse_next", 0.5);

  double Xrob = loadParam(node, "Xrob", 0.0);
  double Yrob = loadParam(node, "Yrob", 0.0);
  double Zrob = loadParam(node, "Zrob", 0.0);
  double qx = loadParam(node, "qx", 0.0);
  double qy = loadParam(node, "qy", 0.0);
  double qz = loadParam(node, "qz", 0.0);
  double qw = loadParam(node, "qw", 1.0);
  double vx0 = loadParam(node, "vx0", 0.0);
  double vy0 = loadParam(node, "vy0", 0.0);
  double vz0 = loadParam(node, "vz0", 0.0);
  double wx0 = loadParam(node, "wx0", 0.0);
  double wy0 = loadParam(node, "wy0", 0.0);
  double wz0 = loadParam(node, "wz0", 0.0);
  int inc = loadParam(node, "inc", 1);
  double cell_size = loadParam(node, "cell_size", 0.2);
  double sph_rad = loadParam(node, "sph_size", 1.0);

  int start_cloud = loadParam(node, "start_cloud", 50);
  int finish_cloud = loadParam(node, "finish_cloud", 300);
  double registration_distance = loadParam(node, "registration_distance", 8.0);
  double max_dis = loadParam(node, "max_dis", 0.005);
  double max_rot = loadParam(node, "max_rot", 0.001);
  double min_dis = loadParam(node, "min_dis", 0.0001);
  double min_rot = loadParam(node, "min_rot", 0.0001);
  int max_iterations = loadParam(node, "max_iterations", 200);
  int max_alignment_iterations = loadParam(node, "max_alignment_iterations", 60);
  double z_map = loadParam(node, "z_map", 0.0);
  int rays = loadParam(node, "rays", 360);
  int rings = loadParam(node, "rings", 32);
  bool correct = loadParam(node, "correct", true);
  double active_points_percent = loadParam(node, "active_points_percent", 0.5);
  std::vector<double> start_mask_angles;
  std::vector<double> finish_mask_angles;
  node.getParam("start_mask_angles", start_mask_angles);
  node.getParam("finish_mask_angles", finish_mask_angles);
  double min_point_part = loadParam(node, "min_point_part", 0.5);
  int max_accum_number = loadParam(node, "max_accum_number", 10);
  //test if memory required is less then 1G
  float mem_req = 1e-9 * (float)(2*Xmax/cell_size*2*Ymax/cell_size*(Zmax-Zmin)/cell_size*sizeof(cell_t));
  ROS_INFO_STREAM("memory requirement "<<mem_req<< " Gbyte");
  if (mem_req > loadParam(node,"max_mem_req", 3) )
  {
    ROS_ERROR_STREAM("memory requirement above req Gig");
    exit(0);
  }

  CloudSparser firstSparser(sparse_first,
                            - Xmax, - Ymax,  Zmin,
                            2 * Xmax, 2 * Ymax, Zmax - Zmin, maxRad, minRad);

  CloudSparser nextSparser(sparse_next,
                              - Xmax, - Ymax,  Zmin,
                              2 * Xmax, 2 * Ymax, Zmax - Zmin, maxRad, minRad);

  CloudNormalizer cloudNorm(rings, rays, start_mask_angles, finish_mask_angles);
  std::ofstream wlog("w.log",std::ofstream::trunc);
  std::ofstream vlog("v.log",std::ofstream::trunc);
  std::ofstream tlog("t.log",std::ofstream::trunc);

//  ATan Atan(rays);
  try
  {
    rosbag::Bag bag;
    bag.open(bag_name, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/velodyne_points"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    ROS_INFO_STREAM("message quantity = "<<view.size());

    rosbag::View::iterator it = view.begin();
    /* advance iterator to start */
    int counter = 0;
    int red_counter = 0;
    for (; (counter < start_cloud) && (it != view.end());
        it ++, counter++)
    {}
    QTransform transf;
    transf.setIdentity();
    transf.setOrigin(tf::Vector3(Xrob, Yrob, Zrob));
    transf.setRotation(tf::Quaternion(qx, qy, qz, qw));
    tf::Vector3 w0(wx0, wy0, wz0);
    QTransform deltaTrans(w0*0.1, Vector3(vx0, vy0, vz0)*0.1);
    //deltaTrans.setIdentity();
    QTransform deltaTransPrev(deltaTrans);
    QTransform oldTrans(transf);
    //in loop transform
    QTransform localTransform;
    localTransform.setIdentity();
    //transform of first cloud
    QTransform firstTransform;
    firstTransform = transf;
    //calculate position and orientation change on each step(velocities)
    //to predict position and orientation on next step
    tf::Vector3 linear_velocity(vx0, vy0, vz0);
    tf::Vector3 angular_velocity(wx0, wy0, wz0);

    std::list<tf::Vector3> linear_velocity_accum;
    std::list<tf::Vector3> angular_velocity_accum;
    int accum_number = 0;

    linear_velocity_accum.push_back(linear_velocity);
    angular_velocity_accum.push_back(angular_velocity);
    accum_number ++;

    ros::WallTime start = ros::WallTime::now();

    //
    double delta_= 2*sph_rad;
    PCRegistration reg(- Xmax-delta_, - Ymax-delta_,  Zmin -delta_,
                       2*Xmax + 2*delta_ , 2*Ymax + 2*delta_, Zmax - Zmin+2*delta_,
                       cell_size, sph_rad,
                       max_iterations,
                       max_dis,    max_rot,
                       min_dis ,    min_rot );
    ros::WallTime end = ros::WallTime::now();
    ROS_INFO_STREAM("creation time = "<< ros::WallDuration((end - start)).toSec());
    ROS_INFO_STREAM("sphere size = "<<reg.getSphere().size());

    //first cloud initialization
    VPointCloud::Ptr cloud = it->instantiate<VPointCloud>();
    if (cloud == NULL)
    {
      ROS_ERROR_STREAM("bag read error");
      return;
    }

    pcl::PCLHeader last_cloud_header = cloud->header;

    ROS_INFO_STREAM("cloud seq = "<<cloud->header.seq<<" time= "<<cloud->header.stamp);
    ROS_INFO_STREAM("initial points quantity = "<<cloud->size());
    double cloud_dt = 0.1; //period between start of cloud and finish of cloud
    double dt = cloud_dt; //period between starts of two subsequent clouds
    if ( correct )
    {
      start = ros::WallTime::now();
      VPointCloud::Ptr normalized_cloud(new VPointCloud);
      tf::Vector3 linear = get_average(linear_velocity_accum);
      tf::Vector3 angular = get_average(angular_velocity_accum);
      ROS_DEBUG_STREAM("linear_avg =  "<<linear);
      ROS_DEBUG_STREAM("angular_avg =  "<<angular);
      size_t normalize_size = cloudNorm.normalize(normalized_cloud,
                                             cloud,
                                    linear, angular,
                                    cloud_dt);
      end = ros::WallTime::now();
      ROS_INFO_STREAM("correction time = "<< ros::WallDuration((end - start)).toSec());
      ROS_INFO_STREAM("correction quantity = "<<normalize_size);
      if ( normalize_size <= normalized_cloud->size()*min_point_part )
      {
        ROS_WARN_STREAM(" bad init cloud normalized size = "<<normalize_size);
        return;
      }
      cloud = normalized_cloud;
    }

    VPointCloud::Ptr first_cloud;

    bool fault = false;
    while (counter < finish_cloud && ! quit && ! fault)
    {
      float time = 0;

      if ( first_cloud )
      {
        time = measure_time(
            boost::bind(&PCRegistration::clearPointCloudFast, &reg, first_cloud) );
//            boost::bind(&PCRegistration::reset, &reg) );
        ROS_INFO_STREAM("map clear time = "<<time);
        //assert( reg.getMap().test_if(is_cell_notzero) == false );
      }

      time = 0;
      localTransform.setIdentity();
      oldTrans.setIdentity();

      firstTransform = transf;

      start = ros::WallTime::now();
      VPointCloud::Ptr sparse_cloud = firstSparser.sparse(cloud);



      time = (ros::WallTime::now() - start).toSec();
      ROS_INFO_STREAM("sparse time = "<<time);
      ROS_INFO_STREAM("sparse quantity = "<<sparse_cloud->size());

      ROS_INFO_STREAM("first cloud seq = "<<cloud->header.seq<<" stamp = "<<cloud->header.stamp);
      time = measure_time(
          boost::bind(
              static_cast<void(PCRegistration::*)(const VPointCloud::Ptr& )>
              (&PCRegistration::setPointCloudFast),
              &reg, boost::ref(sparse_cloud)));
      ROS_INFO_STREAM("first cloud set time = "<<time);
      //remember first cloud for clearing map
      first_cloud = sparse_cloud;
     // publish_map(reg, z_map);
#ifdef SHOW_VIEWER
      {
        VPointCloud::Ptr vCloud(new VPointCloud());
        pcl_ros::transformPointCloud(*cloud, *vCloud, transf.transform());
        int size = 2;
        double r(0.5* (1 + (red_counter%100) / 100.0 ) );
        double g(0), b(0);
        red_counter++;
        //ROS_DEBUG_STREAM("red counter "<<red_counter<<" r = "<<r);
        addSparceCloud2(viewer, vCloud, 0, Xrob - Xmax,Yrob - Ymax, Zrob + Zmin,
                        Xrob + Xmax, Yrob + Ymax, Zrob + Zmax, size, r, g, b);
//        addCloud(viewer, cloud, size, r, g, b);
        showVis(viewer, key_tms);
      }
      cloud_name.clear(); //erase name to fix last cloud in view
#endif


      Vector3 cadr_translation(0,0,0);


      for (int i= 0; /*(i < cnt_max)*/
          !quit && (counter < finish_cloud);
          counter++, i++)
      {
        ROS_DEBUG_STREAM("start cycle "<<counter);
        it++;
        ROS_DEBUG_STREAM("inc iterator "<<it->getDataType());
//        if ( counter == 789)
//          std::cout<<counter<<std::endl;
        if (it == view.end()) //end of bag
          break;
        if (counter % inc != 0)
          continue;
        cloud = it->instantiate<VPointCloud>();
        if (cloud == NULL)
        {
          ROS_ERROR_STREAM("bag read error");
          return;
        }
        //ros::Time cloud_time = pcl_conversions::fromPCL(cloud->header).stamp;
        dt = (cloud->header.stamp - last_cloud_header.stamp)*1.0e-6;
        cloud_dt = dt / (cloud->header.seq - last_cloud_header.seq);
        if (cloud_dt < 0.01)
        {
          cloud_dt = 0.1;
          dt = cloud_dt * (cloud->header.seq - last_cloud_header.seq);
        }
        ROS_INFO_STREAM("new cloud "<<cloud->header.seq);
        ROS_INFO_STREAM("initial points quantity = "<<cloud->size());
        ROS_INFO_STREAM(std::endl<<counter<<" cloud dt = "<<cloud_dt<<" dt = "<<dt);

        //cloud_dt = 0.1;
        int req = 0;
//        std::cin>>req;
        //predict future pose by delta

        QTransform predictTransf = transf*deltaTrans;
        localTransform *= deltaTrans;
#ifdef SHOW_VIEWER
        if (cadr_tms > 0 )
        {
          VPointCloud::Ptr start_cloud(new VPointCloud);
          pcl_ros::transformPointCloud(*cloud, *start_cloud, predictTransf.transform());

          int size = 2;
          double r(0), g(1), b(2);
          changeLastCloud2(viewer, start_cloud, 0, Xrob - Xmax, Yrob - Ymax, Zrob + Zmin, Xrob + Xmax, Yrob + Ymax,
                             Zrob + Zmax, size, r, g, b);
          showVis(viewer, cadr_tms);
        }
#endif
        VPointCloud::Ptr normalized_cloud = cloud;
        if ( correct )
        {
          start = ros::WallTime::now();
          normalized_cloud.reset(new VPointCloud);
          tf::Vector3 linear = get_average(linear_velocity_accum);
          tf::Vector3 angular = get_average(angular_velocity_accum);
          ROS_DEBUG_STREAM("linear_avg =  "<<linear);
          ROS_DEBUG_STREAM("angular_avg =  "<<angular);
          size_t normalize_size = cloudNorm.normalize(normalized_cloud,
                                                      cloud,
                             linear, angular, cloud_dt);
          end = ros::WallTime::now();
          ROS_INFO_STREAM("correction time = "<< ros::WallDuration((end - start)).toSec() <<
                          " correction count = "<<normalize_size <<
                          " from "<<normalized_cloud->size());
          if ( normalize_size <= normalized_cloud->size()*min_point_part )
          {
            ROS_WARN_STREAM(" bad cloud normalized size = "<<normalize_size);
            continue;
          }
          cloud = normalized_cloud;
        }

        start = ros::WallTime::now();
        //sparse
        VPointCloud::Ptr sparse_cloud = //(new VPointCloud);
            nextSparser.sparse(normalized_cloud/*, localTransform*/);
                                                  //localTransform commented because works bad Suddenly

        if ( sparse_cloud->size() == 0)
        {
          ROS_WARN_STREAM("wrong cloud");
          continue;
        }
        time = (ros::WallTime::now() - start).toSec();
        ROS_INFO_STREAM(counter<<" sparse time = "<<time<<" sparse count ="<<sparse_cloud->size());
        last_cloud_header = cloud->header;

#ifdef SHOW_VIEWER
        if (cadr_tms > 0)
        {
          VPointCloud::Ptr start_cloud(new VPointCloud);
          pcl_ros::transformPointCloud(*sparse_cloud, *start_cloud, predictTransf.transform());
          //pcl_ros::transformPointCloud(*cloud, *start_cloud, transf.transform());
          int size = 2;
          double r(0), g(1), b(0);
          changeLastCloud2(viewer, start_cloud, 0, Xrob - Xmax, Yrob - Ymax, Zrob + Zmin, Xrob + Xmax, Yrob + Ymax,
                             Zrob + Zmax, size, r, g, b);
          showVis(viewer, cadr_tms);
        }
#endif

        //registration
        int iterations;
        time = measure_time(
            boost::bind(
                static_cast<int(PCRegistration::*)(const VPointCloud::Ptr&, QTransform& )>
                (&PCRegistration::findTransform),
                &reg, boost::ref(sparse_cloud), boost::ref(localTransform)),
                iterations);
        ROS_INFO_STREAM(counter<<" registration result = "<<iterations<<" time = "<<time);
        tlog<<time<<std::endl;
        if ( iterations <= 0 )
        {
          ROS_INFO_STREAM(" registration fault  ");

          transf = oldTrans * deltaTrans;
          fault = true;
          break;
        }

        //delta = old (-1) * T
        deltaTrans.mult(oldTrans.inverse(), localTransform);
        cadr_translation +=deltaTrans.getOrigin();
        transf = firstTransform * localTransform;//*= deltaTrans;
        print_transform(transf.transform(), "transform");
        print_transform(localTransform.transform(), "localTransform");
        //scale delta to dt - calculate velocity
        linear_velocity = deltaTrans.getOrigin()/dt;
        angular_velocity = deltaTrans.getRotation().getAxis()*
            deltaTrans.getRotation().getAngle()/dt;
        linear_velocity_accum.push_back(linear_velocity);
        angular_velocity_accum.push_back(angular_velocity);
        ++accum_number;
        if ( accum_number > max_accum_number)
        {
          linear_velocity_accum.pop_front();
          angular_velocity_accum.pop_front();
          --accum_number;
        }
        //deltaTrans.getBasis()
        print_transform(deltaTrans.transform(), "delta ");
        ROS_INFO_STREAM("linear velocity = "<<linear_velocity);
        ROS_INFO_STREAM("angular velocity = "<<angular_velocity);
        vlog<<linear_velocity<<std::endl;
        wlog<<angular_velocity<<std::endl;
        publish_twist(deltaTrans);
        oldTrans = localTransform;
#ifdef SHOW_VIEWER
        if ( cadr_result_tms > 0 )
        {
          VPointCloud::Ptr finish_cloud(new VPointCloud);
          //cloud = sparse_cloud;
          pcl_ros::transformPointCloud(*sparse_cloud, *finish_cloud, transf.transform());
//            pcl_ros::transformPointCloud(*cloud, *finish_cloud, transf.transform());
          int size = 2;
          double r(1), g(2), b(5);
          changeLastCloud2(viewer, finish_cloud, 0, Xrob - Xmax, Yrob - Ymax, Zrob + Zmin, Xrob + Xmax, Yrob + Ymax,
                           Zrob + Zmax, size, r, g, b);
          showVis(viewer, cadr_result_tms);
        }
#endif
        if ( iterations >= max_alignment_iterations )
        {
          ROS_INFO_STREAM("iterations above limit - restart alignment");
          break;
        }
        int active_points = reg.activePointsQuantity();
        ROS_DEBUG_STREAM("active points number = "<<
                         active_points<<" percent = "<<(float)active_points/(float)sparse_cloud->size() );
        if ( active_points < sparse_cloud->size()* active_points_percent )
        {
          ROS_INFO_STREAM("active Points at limit "<<reg.activePointsQuantity()<<" - restart alignment");
          break;
        }
        double local_distance = cadr_translation.length();
        if  ( local_distance > registration_distance )
        {
          ROS_INFO_STREAM("local_distance "<<local_distance<<" above limit - restart alignment");
          break;
        }
      } //end for
#ifdef SHOW_VIEWER
      if (!cloud_name.empty())
                viewer->removePointCloud(cloud_name);
#endif
      Xrob = transf.getOrigin().x();
      Yrob = transf.getOrigin().y();
      Zrob = transf.getOrigin().z();
      qx = transf.getRotation().x();
      qy = transf.getRotation().y();
      qz = transf.getRotation().z();
      qw = transf.getRotation().w();
      ROS_INFO_STREAM("rob x = "<<Xrob<<" y = "<<Yrob<<" z = "<<Zrob);
      ROS_INFO_STREAM("rob qx = "<<qx<<" qy = "<<qy<<" qz = "<<qz<<" qw "<<qw);
      //vpause = true;
    } //while
    bag.close();
  }
  catch (rosbag::BagException& e)
  {
    std::cout << "bag error " << e.what() << std::endl;

  }
  catch (ros::Exception& e)
  {
    std::cout << "bag error " << e.what() << std::endl;
  }
  catch (std::exception& e)
  {
    std::cout<<"std error "<<e.what()<<std::endl;
  }
#ifdef SHOW_VIEWER
  if ( !quit )
    showVis(viewer);
#endif
}

int main(int argc, char **argv )
{

  std::cout<<" sph test"<<std::endl;
  ros::init(argc, argv, "sph_test");
//  ROS_INFO_STREAM("info");
//  ROS_DEBUG_STREAM("debug");
//  ROS_ERROR_STREAM("error");
//  std::string log_name("/home/user/local/tmp/2014-12-04-11-30-47.bag");
  //std::string log_name("/media/Acer/ROS/BAG/verstak/2015-10-09-12-02-36.bag");
  //test_SphArr();
  //test_map1<float>();
//  testPCReg();
  //load_from_bag(log_name);
  //load_from_bag_sparse(log_name);
  try
  {
    load_from_bag_sparse_full();
  }
  catch (ros::Exception& e)
  {
    std::cout << "ros error " << e.what() << std::endl;
  }
  //test_laser_data();
  std::cout<<" end"<<std::endl;
}

