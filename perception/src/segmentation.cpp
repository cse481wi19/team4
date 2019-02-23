#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"

#include <math.h>
#include <sstream>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices,
	                           PointCloudC::Ptr subset_cloud) { // changed method signature!!! add one more param
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
  pcl::ModelCoefficients coeff;
  seg.segment(indices_internal, coeff);

  *indices = indices_internal;

  if (indices->indices.size() == 0) {
    ROS_ERROR("Unable to find surface.");
    return;
  }


  // reify a point cloud from its indices
  // Extract subset of original_cloud into subset_cloud:
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.filter(*subset_cloud);
}

// Lab 31
// // use indices to access planar surface
// PointCloudC::Ptr cloud(new PointCloudC);
// pcl::PointIndices indices;
// for (size_t i=0; i<indices.indices.size(); ++i) {
//   int index = indices.indices[i];
//   const PointC& pt = cloud->points[index];
// }

// 

Segmenter::Segmenter(const ros::Publisher& surface_points_pub)
    : surface_points_pub_(surface_points_pub) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  PointCloudC::Ptr subset_cloud(new PointCloudC);
  SegmentSurface(cloud, table_inliers, subset_cloud);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*subset_cloud, msg_out);
  surface_points_pub_.publish(msg_out);
}
}  // namespace perception