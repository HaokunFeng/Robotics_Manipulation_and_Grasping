#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

#include "perception/object.h"
#include "perception/object_recognizer.h"

namespace perception {

// // Does a complete bin segmentation pipeline.
// //
// // Args:
// //  cloud: The point cloud with the bin and the objects in it.
// //  objects: The output objects.
// void SegmentObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
//                           std::vector<Object>* objects);

// Finds the largest horizontal surface in the given point cloud.
// This is useful for adding a collision object to MoveIt.
//
// Args:
//  cloud: The point cloud to extract a surface from.
//  indices: The indices of points in the point cloud that correspond to the
//    surface. Empty if no surface was found.
void SegmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    pcl::PointIndices::Ptr indices, 
                    pcl::ModelCoefficients::Ptr coeff);

// Segments the objects above a surface.
//
// Args:
//  cloud: The point cloud with the surface and the objects above it.
//  surface_indices: The indices corresponding to the surface.
//  object_indices: A vector of PointIndices, where each entry in the vector
//    corresponds to one object. Each PointIndices contains the indices in the
//    point cloud corresponding that that object.
void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices);


// Does a complete tabletop segmentation pipeline.
//
// Args:
//  cloud: The point cloud with the surface and the objects above it.
//  objects: The output objects.
void SegmentTabletopScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          std::vector<Object>* objects);


// Computes the axis-aligned bounding box of a point cloud.
//
// Args:
//  cloud: The point cloud
//  pose: The output pose. Because this is axis-aligned, the orientation is just
//    the identity. The position refers to the center of the box.
//  dimensions: The output dimensions, in meters.
void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions);


class Segmenter {
 public:
  Segmenter(const ros::Publisher& surface_points_pub,
            const ros::Publisher& above_surface_pub,
            const ros::Publisher& marker_pub,
            const ObjectRecognizer& recognizer);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher surface_points_pub_;
  ros::Publisher above_surface_pub_;
  ros::Publisher marker_pub_;
  ObjectRecognizer recognizer_;
};
}  // namespace perception_new



/*
#ifndef PERCEPTION_SEGMENTATION_H
#define PERCEPTION_SEGMENTATION_H

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/extract_indices.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"

namespace perception {

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions);

void SegmentBinObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                       std::vector<pcl::PointIndices>* indices);

class Segmenter {
public:
    Segmenter(const ros::Publisher& points_pub);
    void Callback(const sensor_msgs::PointCloud2& msg);

private:
    ros::Publisher points_pub_;
    ros::Publisher marker_pub_;
};

}  // namespace perception

#endif // PERCEPTION_SEGMENTATION_H
*/