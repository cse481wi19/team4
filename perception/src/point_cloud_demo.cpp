#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::spin();
  return 0;
}