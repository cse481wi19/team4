#include "perception/crop.h"
#include "perception/downsample.h"
#include "perception/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "visualization_msgs/Marker.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;

  // the commented part is the previous implement from lab 30 (changed to current from lab 31)
  
  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);
  ros::Subscriber crop_sub =
      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);

//   ros::NodeHandle nh_down;
//   ros::Publisher down_pub =
//       nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
//   perception::Downsampler downsampler(down_pub);
//   ros::Subscriber down_sub =
//       nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);

  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  ros::Publisher above_surface_pub = 
      nh.advertise<sensor_msgs::PointCloud2>("above_surface_pub", 1, true);

  perception::Segmenter segmenter(table_pub, marker_pub, above_surface_pub);
  
  ros::Subscriber sub =
      nh.subscribe("cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);


  ros::spin();
  return 0;
}