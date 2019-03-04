#include "perception/segmentation.h"
#include "perception/box_fitter.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/common/angles.h"
#include "pcl/common/common.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "visualization_msgs/Marker.h"
#include "perception/downsample.h"
#include "pcl/filters/voxel_grid.h"
#include "perception/object.h"

#include "geometry_msgs/Pose.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"

#include <math.h>
#include <sstream>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {

// void SegmentSurface(PointCloudC::Ptr cloud, 
//                     pcl::PointIndices::Ptr indices) {
// lab 33
void SegmentSurface(PointCloudC::Ptr cloud, 
                    pcl::PointIndices::Ptr indices,
                    pcl::ModelCoefficients::Ptr coeff) { 
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);
  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
  axis << 0, 0, 1;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(10.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  // pcl::ModelCoefficients coeff;
  seg.segment(indices_internal, *coeff);

  double distance_above_plane;
  ros::param::param("distance_above_plane", distance_above_plane, 0.005);

  // Build custom indices that ignores points above the plane.
  for (size_t i = 0; i < cloud->size(); ++i) {
    const PointC& pt = cloud->points[i];
    float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
                coeff->values[2] * pt.z + coeff->values[3];
    if (val <= distance_above_plane) {
      indices->indices.push_back(i);
    }
  }

  *indices = indices_internal;
  if (indices->indices.size() == 0) {
    ROS_ERROR("Unable to find surface.");
    return;
  }
}

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
  PointC min_pcl, max_pcl;

  pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);

  pose->position.x = (min_pcl.x + max_pcl.x) / 2;
  pose->position.y = (min_pcl.y + max_pcl.y) / 2;
  pose->position.z = (min_pcl.z + max_pcl.z) / 2;

  dimensions->x = (max_pcl.x - min_pcl.x);
  dimensions->y = (max_pcl.y - min_pcl.y);
  dimensions->z = (max_pcl.z - min_pcl.z);
}

void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices,
                           const ros::Publisher& marker_pub_p,
                           pcl::ModelCoefficients::Ptr coeff) {
  // extract the scene above the plane
  pcl::ExtractIndices<PointC> extract;
  pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
  extract.setInputCloud(cloud);
  extract.setIndices(surface_indices);
  extract.setNegative(true);
  extract.filter(above_surface_indices->indices);

  ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

  // implement the actual clustering
  double cluster_tolerance;
  int min_cluster_size, max_cluster_size;
  ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
  ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
  ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setIndices(above_surface_indices);
  euclid.setClusterTolerance(cluster_tolerance);
  euclid.setMinClusterSize(min_cluster_size);
  euclid.setMaxClusterSize(max_cluster_size);
  euclid.extract(*object_indices);
 
  // Find the size of the smallest and the largest object,
  // where size = number of points in the cluster
  size_t min_size = std::numeric_limits<size_t>::max();
  size_t max_size = std::numeric_limits<size_t>::min();
  for (size_t i = 0; i < object_indices->size(); ++i) {
    size_t cluster_size = object_indices->at(i).indices.size();
    if (cluster_size > max_size) {
      max_size = cluster_size;
    }
    if (cluster_size < min_size) {
      min_size = cluster_size;
    }
  }

  ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
          object_indices->size(), min_size, max_size);

  // // PREV VERSION
  // // draw boxes around each of the objects
  // for (size_t i = 0; i < object_indices->size(); ++i) {
  //   // Reify indices into a point cloud of the object.
  //   pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  //   *indices = object_indices->at(i);
  //   PointCloudC::Ptr object_cloud(new PointCloudC());
  //   // TODO: fill in object_cloud using indices
  //   extract.setNegative(false);
  //   extract.setIndices(indices);
  //   extract.filter(*object_cloud);

  //   // Publish a bounding box around it.
  //   visualization_msgs::Marker object_marker;
  //   object_marker.ns = "objects";
  //   object_marker.id = i;
  //   object_marker.header.frame_id = "base_link";
  //   object_marker.type = visualization_msgs::Marker::CUBE;
  //   GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
  //                             &object_marker.scale);
  //   object_marker.color.g = 1;
  //   object_marker.color.a = 0.3;
  //   marker_pub_p.publish(object_marker);
  // }
  // // PREV VERSION
}

void SegmentTabletopScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          std::vector<Object>* objects,
                          const ros::Publisher& marker_pub_p,
                          const ros::Publisher& surface_points_pub,
                          const ros::Publisher& above_surface_pub) {
  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  PointCloudC::Ptr subset_cloud(new PointCloudC);
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  pcl::ExtractIndices<PointC> extract;
  SegmentSurface(cloud, table_inliers, coeff);
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.filter(*subset_cloud);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*subset_cloud, msg_out);
  if (&surface_points_pub != &NULL_P) {
    surface_points_pub.publish(msg_out);
  }

  std::vector<pcl::PointIndices> object_indices;
  SegmentSurfaceObjects(cloud, table_inliers, &object_indices, marker_pub_p, coeff);
  // We are reusing the extract object created earlier in the callback.
  PointCloudC::Ptr cloud_out(new PointCloudC());
  extract.setNegative(true);
  extract.filter(*cloud_out);
  pcl::toROSMsg(*cloud_out, msg_out);
  if (&above_surface_pub != &NULL_P) {
    above_surface_pub.publish(msg_out);
  }

  // extract.setNegative(true); // new
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
    FitBox(*object_cloud, coeff, *extract_out, shape, object_pose);
        
    // perception::Object new_object();
    perception::Object *object_i = new perception::Object;
    object_i->cloud = object_cloud;
    object_i->pose = object_pose;
    object_i->dimensions.x = shape.dimensions[0];
    object_i->dimensions.y = shape.dimensions[1];
    object_i->dimensions.z = shape.dimensions[2];

    objects->push_back(*object_i);

    if (&marker_pub_p != &NULL_P) {
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
    }
  }

  // for (size_t i = 0; i < objects.size(); ++i) {
  //   const Object& object = objects[i];

  //   // Publish a bounding box around it.
  //   visualization_msgs::Marker object_marker;
  //   object_marker.ns = "objects";
  //   object_marker.id = i;
  //   object_marker.header.frame_id = "base_link";
  //   object_marker.type = visualization_msgs::Marker::CUBE;
  //   object_marker.pose = object.pose;
  //   object_marker.scale = object.dimensions;
  //   object_marker.color.g = 1;
  //   object_marker.color.a = 0.3;
  //   marker_pub_.publish(object_marker);
  // }
}

Segmenter::Segmenter(const ros::Publisher& surface_points_pub,
                     const ros::Publisher& marker_pub,
                     const ros::Publisher& above_surface_pub)
    : surface_points_pub_(surface_points_pub),
      marker_pub_(marker_pub),
      above_surface_pub_(above_surface_pub) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  // LAB 30~33
  // PointCloudC::Ptr cloud(new PointCloudC());
  // pcl::fromROSMsg(msg, *cloud);

  // pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  // PointCloudC::Ptr subset_cloud(new PointCloudC);
  // pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  // pcl::ExtractIndices<PointC> extract;
  // SegmentSurface(cloud, table_inliers, coeff);
  // extract.setInputCloud(cloud);
  // extract.setIndices(table_inliers);
  // extract.filter(*subset_cloud);

  // sensor_msgs::PointCloud2 msg_out;
  // pcl::toROSMsg(*subset_cloud, msg_out);
  // surface_points_pub_.publish(msg_out);

  // visualization_msgs::Marker table_marker;
  // table_marker.ns = "table";
  // table_marker.header.frame_id = "base_link";
  // table_marker.type = visualization_msgs::Marker::CUBE;
  // lab 33 - remove table marker, use extractShape instead to get the bouding box for table
  // GetAxisAlignedBoundingBox(subset_cloud, &table_marker.pose, &table_marker.scale);
  // table_marker.color.r = 1;
  // table_marker.color.a = 0.8;
  // marker_pub_.publish(table_marker);

  // lab 33
  // visualize the table
  // PointCloudC::Ptr extract_out(new PointCloudC());
  // shape_msgs::SolidPrimitive shape;
  // geometry_msgs::Pose table_pose;
  // simple_grasping::extractShape(*subset_cloud, coeff, *extract_out, shape,
  //                               table_pose);
  // if (shape.type == shape_msgs::SolidPrimitive::BOX) {
  //   // make sure shape is a box before looking at its dimensions
  //   table_marker.scale.x = shape.dimensions[0];
  //   table_marker.scale.y = shape.dimensions[1];
  //   table_marker.scale.z = shape.dimensions[2];
  //   // shift the box height back by 1
  //   table_marker.pose = table_pose;
  //   // table_marker.pose.position.z -= table_marker.scale.z;

  //   // publish the marker
  //   table_marker.color.r = 1;
  //   table_marker.color.a = 0.8;
  //   marker_pub_.publish(table_marker);
  // }

  // std::vector<pcl::PointIndices> object_indices;
  // SegmentSurfaceObjects(cloud, table_inliers, &object_indices, marker_pub_, coeff);

  // // We are reusing the extract object created earlier in the callback.
  // PointCloudC::Ptr cloud_out(new PointCloudC());
  // extract.setNegative(true);
  // extract.filter(*cloud_out);
  // pcl::toROSMsg(*cloud_out, msg_out);
  // above_surface_pub_.publish(msg_out);
  // LAB 30~33


  // Lab34
  PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud_unfiltered);
  
  PointCloudC::Ptr cloud(new PointCloudC());
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

  // std::vector<Object> objects;
  std::vector<Object> objects;
  SegmentTabletopScene(cloud, &objects, marker_pub_, surface_points_pub_, above_surface_pub_);
}
}  // namespace perception