#include <vector>

#include "perception/crop.h"
#include "perception/foodSegmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "visualization_msgs/Marker.h"

#include "perception/object_recognizer.h"
#include "perception_msgs/ObjectFeatures.h"
#include "perception_msgs/ObjectPosition.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    ROS_INFO("Usage: rosrun perception food_detection DATA_DIR");
    ros::spinOnce();
  }
  std::string data_dir(argv[1]);

  ros::init(argc, argv, "food_detector");
  ros::NodeHandle nh;
  
  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
//   perception::Cropper cropper(crop_pub);
//   ros::Subscriber crop_sub =
//       nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);

  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  ros::Publisher above_surface_pub = 
      nh.advertise<sensor_msgs::PointCloud2>("above_surface_pub", 1, true);
  ros::Publisher food_pub = 
      nh.advertise<perception_msgs::ObjectPosition>("food_pub", 1, true);

//   std::vector<perception_msgs::ObjectFeatures> dataset;
//   perception::LoadData(data_dir, &dataset);
//   perception::ObjectRecognizer recognizer(dataset);

//   perception_food::FoodDetector detector(table_pub, marker_pub, above_surface_pub, food_pub, recognizer);
  
//   ros::Subscriber sub =
//       nh.subscribe("cropped_cloud", 1, &perception_food::FoodDetector::Callback, &detector);

  perception::Cropper Cropper(crop_pub);


  ros::spin();
  return 0;
}