/*
 * viewer.cpp
 *
 *  Created on: 7 нояб. 2016 г.
 *      Author: user
 */


#include "viewer.h"
using namespace registration;


Viewer::Viewer(const std::string& name ) :
    viewer(new pcl::visualization::PCLVisualizer(name)), cloud_count(0), vpause(false), stop(false), quit(false)
{
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  conn = viewer->registerKeyboardCallback(keyboardEventOccurred, this);
}

Viewer:: ~Viewer()
{
    conn.disconnect();
}

void Viewer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event)
{
  if (event.keyDown())
  {
    std::cout << "key was pressed " << std::endl;

    switch (event.getKeyCode())
    {
      case 'q':
      case 'x':
        std::cout << " quit " << std::endl;
        quit = true;
        break;
      case 'p':
        std::cout << " pause " << std::endl;
        vpause = !vpause;
        break;

      default:
        break;
    }

    stop = true;
  }
}

void Viewer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                                   void* viewer_void)
{
  Viewer* v = static_cast<Viewer *>(viewer_void);
  v->keyboardEventOccurred(event);

}

void Viewer::addCloud(const VPointCloud::ConstPtr& cloud,
                      double rad , double r , double g , double b)
{
  cloud_count++;
  std::stringstream ss;
  ss << "cloud_" << cloud_count;
  //  ROS_INFO_STREAM("add cloud "<<ss.str());
  cloud_name = (ss.str());
  viewer->addPointCloud < VPoint > (cloud, cloud_name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, rad, cloud_name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cloud_name);
}

void Viewer::addRGBCloud(const RGBCloud::ConstPtr& cloud, double rad )
{
  cloud_count++;
  std::stringstream ss;
  ss << "cloud_" << cloud_count;
  //  ROS_INFO_STREAM("add cloud "<<ss.str());
  cloud_name = (ss.str());
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud < pcl::PointXYZRGB > (cloud, rgb, cloud_name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, rad, cloud_name);
}

void Viewer::addIRGBCloud(const VPointCloud::ConstPtr& cloud,
                          double r, double g,  double b, double rad)
{
  //VPointCloud::Ptr cloud_res(new VPointCloud);
  RGBCloud::Ptr cloud_res(new RGBCloud);
  cloud_res->reserve(cloud->size());
  for (int i = 0; i < cloud->size(); ++i)
  {
    const VPoint& p = (*cloud)[i];

    pcl::PointXYZRGB out_p;
    out_p.x = p.x;
    out_p.y = p.y;
    out_p.z = p.z;
    float intensity = p.intensity;
    out_p.r = r * intensity*10;
    out_p.g = g * intensity*10;
    out_p.b = b * intensity*10;
    cloud_res->push_back(out_p);
    //cloud_res->push_back(p);
  }
  //addCloud(viewer, cloud_res, size, r, g, b);
  addRGBCloud(cloud_res, rad);
}

void Viewer::addHRGBCloud(const VPointCloud::ConstPtr& cloud, double min, double max, double rad)
{
    RGBCloud::Ptr cloud_res(new RGBCloud);
    cloud_res->reserve(cloud->size());
    for (int i = 0; i < cloud->size(); ++i)
    {
      const VPoint& p = (*cloud)[i];
      if ( p.z > max || p.z < min )
          continue;
      pcl::PointXYZRGB out_p;
      out_p.x = p.x;
      out_p.y = p.y;
      out_p.z = p.z;
      float val = (p.z - min)/ (max - min);
      if (val < 1.0/3.0)
      {
          out_p.r = 255*3*(1.0/3.0 - val);
          out_p.g = 255*3*(val);
          out_p.b = 0;
      }else if(val < 2.0/3.0)
      {
          out_p.r = 0;
          out_p.g = 255*3*(2.0/3.0 - val);
          out_p.b = 255*3*(val - 1.0/3.0);
      }else
      {
          out_p.r = 255*3*(val - 2.0/3.0);
          out_p.g = 0;
          out_p.b = 255*3*(1.0 - val);
      }
      cloud_res->push_back(out_p);
      //cloud_res->push_back(p);
    }
    //addCloud(viewer, cloud_res, size, r, g, b);
    addRGBCloud(cloud_res, rad);
}

void Viewer::addHRGBCloud(const VPointCloud::ConstPtr& cloud,
                  const VPointCloud::ConstPtr& orig_cloud,
                  double min, double max, double rad)
{
    RGBCloud::Ptr cloud_res(new RGBCloud);
    assert(cloud->size() == orig_cloud->size());
    cloud_res->reserve(cloud->size());
    for (int i = 0; i < cloud->size(); ++i)
    {
      const VPoint& p = (*cloud)[i];
      const VPoint& o_p = (*orig_cloud)[i];
      if ( o_p.z > max || o_p.z < min )
          continue;
      pcl::PointXYZRGB out_p;
      out_p.x = p.x;
      out_p.y = p.y;
      out_p.z = p.z;
      float val = (o_p.z - min)/ (max - min);
      if (val < 1.0/3.0)
      {
          out_p.r = 255*3*(1.0/3.0 - val);
          out_p.g = 255*3*(val);
          out_p.b = 0;
      }else if(val < 2.0/3.0)
      {
          out_p.r = 0;
          out_p.g = 255*3*(2.0/3.0 - val);
          out_p.b = 255*3*(val - 1.0/3.0);
      }else
      {
          out_p.r = 255*3*(val - 2.0/3.0);
          out_p.g = 0;
          out_p.b = 255*3*(1.0 - val);
      }
      cloud_res->push_back(out_p);
      //cloud_res->push_back(p);
    }
    //addCloud(viewer, cloud_res, size, r, g, b);
    addRGBCloud(cloud_res, rad);
}


void Viewer::show()
{

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

void Viewer::show(int tms)
{

  while ((!viewer->wasStopped() && tms != 0) || vpause)
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    if (tms <= 0)
    {
      if (stop)
        break;
    }
    else
    {
      tms -= 100;
    }

  }
  stop = false;
  if (quit)
  {
    viewer->close();
    //exit(0);
  }


}

void Viewer::showOnce(int tms)
{
    viewer->spinOnce(tms);
}





