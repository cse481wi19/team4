#include <vector>

#include "perception/crop.h"
#include "perception/foodSegmentation.h"
#include "pcl_ros/transforms.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "visualization_msgs/Marker.h"

#include "perception/object_recognizer.h"
#include "perception_msgs/ObjectFeatures.h"
#include "perception_msgs/ObjectPosition.h"

int main(int argc, char** argv) {
  // if (argc < 2) {
  //   ROS_INFO("Usage: rosrun perception food_detection DATA_DIR");
  //   ros::spinOnce();
  // }
  // std::string data_dir(argv[1]);

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

  std::vector<perception_msgs::ObjectFeatures> dataset;
//   perception::LoadData(data_dir, &dataset);
  perception::ObjectRecognizer recognizer(dataset);

//   perception_food::FoodDetector detector(table_pub, marker_pub, above_surface_pub, food_pub, recognizer);
  
//   ros::Subscriber sub =
//       nh.subscribe("cropped_cloud", 1, &perception_food::FoodDetector::Callback, &detector);

  perception::Cropper cropper(crop_pub);
  perception_food::FoodDetector detector(table_pub, marker_pub, above_surface_pub, food_pub, recognizer);
//   ros::Subscriber sub =
//       nh.subscribe("cropped_cloud", 1, &perception_food::FoodDetector::Callback, &detector);


  boost::shared_ptr<const geometry_msgs::Pose> object_position_message;
  boost::shared_ptr<const sensor_msgs::PointCloud2> cloud_msg;
  while (true) {
    object_position_message = ros::topic::waitForMessage<geometry_msgs::Pose>("feed_pub");
    if (object_position_message != NULL) {
        ROS_INFO("[food_detection.cpp] Got a request for a object location!");
        // get next message from input camera (cloud_in) and pass it to Cropper callback
        cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_in"); // block
        tf::TransformListener tf_listener;                                                   
        tf_listener.waitForTransform("base_link", cloud_msg->header.frame_id,                     
                                    ros::Time(0), ros::Duration(5.0)); 
        std::cerr <<  cloud_msg->header.frame_id << std::endl;                      
        tf::StampedTransform transform;                                                       
        try {                                                                                 
            tf_listener.lookupTransform("base_link", cloud_msg->header.frame_id,                    
                                        ros::Time(0), transform);                               
        } catch (tf::LookupException& e) {                                                    
            std::cerr << e.what() << std::endl;                                                 
            return 1;                                                                           
        } catch (tf::ExtrapolationException& e) {                                             
            std::cerr << e.what() << std::endl;                                                 
            return 1;                                                                           
        }                                                                                                                                                                 
        sensor_msgs::PointCloud2 cloud_out;                                                   
        pcl_ros::transformPointCloud("base_link", transform, *cloud_msg, cloud_out);
        // if (cloud_out != NULL) {
            ROS_INFO("[food_detection.cpp] Got the cloud_msg; calling cropper");
            // for some reason segmenter callback not picking up published cropped cloud - doing it manually
            sensor_msgs::PointCloud2 cropped_msg = cropper.Call(cloud_out); //cloud_out);

            detector.Callback(cropped_msg);
        // } else {
        //     ROS_INFO("Error - didn't get an input msg from camera! Returning 'no balls found'");
        //     perception_msgs::ObjectPosition object_position_msg;
        //     food_pub.publish(object_position_msg);
        // }
        // set reset pointers just in case (boost pointers dont let you just set them to null)
        object_position_message.reset();
        cloud_msg.reset();
    }
  }


  ros::spin();
  return 0;
}