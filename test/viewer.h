/*
 * viewer.h
 *
 *  Created on: 8 нояб. 2016 г.
 *      Author: user
 */

#ifndef TEST_VIEWER_H_
#define TEST_VIEWER_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>

#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include "vcloud.h"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewPtr;

using registration::VPoint;
using registration::VPointCloud;
using registration::RGBCloud;

class Viewer
{
protected:
  ViewPtr viewer;
  int cloud_count;
  std::string cloud_name;
  bool vpause;
  bool stop;
  bool quit;
  boost::signals2::connection conn;
  static void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                                void* viewer_void);
public:
  Viewer(const std::string& name = "3D Viewer");
  virtual ~Viewer();

  virtual void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event);



  void addCloud(const VPointCloud::ConstPtr& cloud, double rad =1.0 ,
              double r = 1.0 , double g = 1.0,  double b = 1.0);
    //add cloud with color proportional to intensity
  void addIRGBCloud(const VPointCloud::ConstPtr& cloud,
                    double r = 1.0 , double g = 1.0,  double b = 1.0, double rad =1.0);
    //add cloud with color proportional to height
  void addHRGBCloud(const VPointCloud::ConstPtr& cloud, double min, double max, double rad =1.0);
  //add cloud with color proportional to height of original cloud
  void addHRGBCloud(const VPointCloud::ConstPtr& cloud,
                    const VPointCloud::ConstPtr& orig_cloud,
                    double min, double max, double rad =1.0);

  void addRGBCloud(const RGBCloud::ConstPtr& cloud, double rad =1.0 );
  void show();

  void show(int tms);

  void showOnce(int tms);
};




#endif /* TEST_VIEWER_H_ */
