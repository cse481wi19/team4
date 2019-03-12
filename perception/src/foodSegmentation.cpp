#include "perception/foodSegmentation.h"
#include "segmentation.cpp"

#include "perception_msgs/ObjectPosition.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception_food {

void SegmentFoodScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          std::vector<perception::Object>* objects,
                          const ros::Publisher& marker_pub_p,
                          const ros::Publisher& surface_points_pub,
                          const ros::Publisher& above_surface_pub,
                          const ros::Publisher& food_pub,
                          perception::ObjectRecognizer& recognizer) {
  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  PointCloudC::Ptr subset_cloud(new PointCloudC);
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  pcl::ExtractIndices<PointC> extract;
  perception::SegmentSurface(cloud, table_inliers, coeff);
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.filter(*subset_cloud);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*subset_cloud, msg_out);
  surface_points_pub.publish(msg_out);

  std::vector<pcl::PointIndices> object_indices;
  perception::SegmentSurfaceObjects(cloud, table_inliers, &object_indices, coeff);
  // We are reusing the extract object created earlier in the callback.
  PointCloudC::Ptr cloud_out(new PointCloudC());
  extract.setNegative(true);
  extract.filter(*cloud_out);
  pcl::toROSMsg(*cloud_out, msg_out);
  above_surface_pub.publish(msg_out);

  extract.filter(*subset_cloud); // new
  // draw boxes around each of the objects
  for (size_t i = 0; i < object_indices.size(); ++i) {
    // Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices.at(i);
    PointCloudC::Ptr object_cloud(new PointCloudC());

    // fill in object_cloud using indices
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*object_cloud);
    
    PointCloudC::Ptr extract_out(new PointCloudC());
    shape_msgs::SolidPrimitive shape;
    geometry_msgs::Pose object_pose;
    perception::FitBox(*object_cloud, coeff, *extract_out, shape, object_pose);
        
    perception::Object *object_i = new perception::Object;
    object_i->cloud = object_cloud;
    object_i->pose = object_pose;
    object_i->dimensions.x = shape.dimensions[0];
    object_i->dimensions.y = shape.dimensions[1];
    object_i->dimensions.z = shape.dimensions[2];
    std::cerr << "x " << shape.dimensions[0] << std::endl;
    std::cerr << "y " << shape.dimensions[1] << std::endl;
    std::cerr << "z " << shape.dimensions[2] << std::endl;
    // ROS_INFO("x: %f, y: %f, z: %f", object_i->pose.position.x, object_i->pose.position.y, object_i->pose.position.z);
    objects->push_back(*object_i);

    float x = shape.dimensions[0];
    float y = shape.dimensions[1];
    float z = shape.dimensions[2];
    if (checkRange(x, y, z)) {
        // Publish a bounding box around it.
        visualization_msgs::Marker object_marker;
        object_marker.ns = "objects";
        object_marker.id = i;
        object_marker.header.frame_id = "base_link";
        object_marker.type = visualization_msgs::Marker::CUBE;
        object_marker.pose = object_i->pose;
        object_marker.scale = object_i->dimensions;
        object_marker.color.g = 1;
        object_marker.color.a = 0.3;
        marker_pub_p.publish(object_marker);

         ////////////////////////////////////////////////////////////////////////////////////
        // Recognize the object
        // std::string name;
        // double confidence;
        // // recognize the object with the recognizer_
        // const perception::Object& object_const = *object_i;
        // recognizer.Recognize(object_const, &name, &confidence);
        // confidence = round(1000 * confidence) / 1000;

        // std::stringstream ss;
        // ss << name << " (" << confidence << ")";

        // // Publish the recognition result as a marker
        // visualization_msgs::Marker name_marker;
        // name_marker.ns = "recognition";
        // name_marker.id = i;
        // name_marker.header.frame_id = "base_link";
        // name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        // name_marker.pose.position = object_i->pose.position;
        // name_marker.pose.position.z += 0.1;    
        // name_marker.pose.orientation.w = 1;
        // name_marker.scale.x = 0.025;
        // name_marker.scale.y = 0.025;
        // name_marker.scale.z = 0.025;
        // name_marker.color.r = 0;
        // name_marker.color.g = 0;
        // name_marker.color.b = 1.0;
        // name_marker.color.a = 1.0;
        // name_marker.text = ss.str();
        // marker_pub_p.publish(name_marker);

        // publish the recognition result as a msg
        perception_msgs::ObjectPosition position;
        position.axis.push_back("x");
        position.values.push_back(object_i->pose.position.x);
        position.axis.push_back("y");
        position.values.push_back(object_i->pose.position.y);
        position.axis.push_back("z");
        position.values.push_back(object_i->pose.position.z);
        food_pub.publish(position);
    }
  }
}

bool checkRange(const float x, const float y, const float z) {
    bool ret = OBJ_X_MIN <= x && x <= OBJ_X_MAX;
    ret &= OBJ_Y_MIN <= y && y <= OBJ_Y_MAX;
    ret &= OBJ_Z_MIN <= z && z <= OBJ_Z_MAX;
    return ret;
}

FoodDetector::FoodDetector(const ros::Publisher& surface_points_pub,
                     const ros::Publisher& marker_pub,
                     const ros::Publisher& above_surface_pub,
                     const ros::Publisher& food_pub,
                     const perception::ObjectRecognizer& recognizer)
    : surface_points_pub_(surface_points_pub),
      marker_pub_(marker_pub),
      above_surface_pub_(above_surface_pub),
      food_pub_(food_pub), 
      recognizer_(recognizer) {}

void FoodDetector::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud_unfiltered);
  
  PointCloudC::Ptr cloud(new PointCloudC());
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

  std::vector<perception::Object> objects;
  SegmentFoodScene(cloud, &objects, marker_pub_, surface_points_pub_, above_surface_pub_, food_pub_, recognizer_);
}
}  // namespace perception_food