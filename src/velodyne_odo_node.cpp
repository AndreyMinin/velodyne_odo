/*
 * velodyne_odo_node_mt.cpp
 *
 *  Created on: 10.05.2016
 *      Author: user
 */

#include <ros/ros.h>
#include "VelodyneOdo.h"

/** Main entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "heightmap_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");


  registration::VelodyneOdo vo(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}



