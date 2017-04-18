#include "cloud_viewer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "viewer");
  ros::NodeHandle nh("~");

  CloudViewer CV(nh);

  CV.run();


}
