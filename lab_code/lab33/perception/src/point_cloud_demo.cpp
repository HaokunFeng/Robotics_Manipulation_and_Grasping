#include <vector>

#include "perception/crop.h"
#include "perception/object_recognizer.h"
#include "perception/segmentation.h"
#include "perception_msgs/ObjectFeatures.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;

  if (argc < 2) {
    ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
    ros::spinOnce();
  }
  std::string data_dir(argv[1]);

  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);
  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);

  ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  ros::Publisher above_table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("above_table_cloud", 1, true);
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);

  // Create the object recognizer.
  std::vector<perception_msgs::ObjectFeatures> dataset;
  perception::LoadData(data_dir, &dataset);
  perception::ObjectRecognizer recognizer(dataset);

  perception::Segmenter segmenter(table_pub, above_table_pub, marker_pub,
                                  recognizer);
  ros::Subscriber segment_sub = nh.subscribe(
      "cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);

  ros::spin();
  return 0;
}





// #include "perception/crop.h"
// #include "perception/downsample.h"
// #include "ros/ros.h"
// #include "sensor_msgs/PointCloud2.h"
// #include "perception/segmentation.h"

// // includes for service callback testing
// //#include "perception/GetSpineLocations.h"
// #include "geometry_msgs/Pose.h"

// // For the service callback testing

// // bool return_pose(perception_new::GetSpineLocations::Request &req, perception_new::GetSpineLocations::Response &res) {
// //     ROS_INFO("Recieved request");
// //     geometry_msgs::Pose book_pose;
// //     res.spine_poses.push_back(book_pose);
// //     return true;
// // }



// int main(int argc, char** argv) {
//   ros::init(argc, argv, "point_cloud_demo");
//   ros::NodeHandle nh;

//   ROS_INFO("Here");
  
//   // Lab 30 cropping / downsampling
//   ros::Publisher crop_pub =
//       nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
//   ros::Publisher down_pub =
//       nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
//   perception::Cropper cropper(crop_pub);
//   perception::Downsampler downsampler(down_pub);
//   ros::Subscriber sub =
//       nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
//   ros::Subscriber sub2 =
//       nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);

//   // Publisher for the segmented table cloud
//   ros::Publisher table_pub =
//       nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);

//   // Publisher for the visualization markers
//   ros::Publisher marker_pub =
//       nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

//   // Publisher for the clou made up of objects aboce the table
//   ros::Publisher above_table_pub = nh.advertise<sensor_msgs::PointCloud2>("above_table_cloud", 1, true);

//   // The segmentor class that takes a point cloud and divides up the surface and objects
//   perception::Segmenter segmenter(table_pub, marker_pub, above_table_pub); 
//   ros::Subscriber sub3 =
//     nh.subscribe("cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);

//   ROS_INFO("Here2");

//   // Test code for the service call
//   //ros::ServiceServer service = nh.advertiseService("get_spines_test", return_pose);


//   ros::spin();
//   return 0;
// }



/*
#include "perception/crop.h"
#include "perception/downsample.h"
#include "perception/segmentation.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  // perception::Cropper cropper;

  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);
  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
  
  ros::Publisher downsample_pub =
      nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
  perception::Downsampler downsampler(downsample_pub);
  ros::Subscriber sub2 =
      nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);

  ros::Publisher segment_pub =
      nh.advertise<sensor_msgs::PointCloud2>("segment_cloud", 1, true);
  perception::Segmenter segmenter(segment_pub);
  ros::Subscriber sub3 =
      nh.subscribe("cloud_in", 1, &perception::Segmenter::Callback, &segmenter);
  
  ros::spin();
  return 0;
}
*/