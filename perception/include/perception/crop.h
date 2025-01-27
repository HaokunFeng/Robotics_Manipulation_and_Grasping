#ifndef PERCEPTION_CROP_H
#define PERCEPTION_CROP_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace perception {
    // class Cropper {
        // public:
        //     Cropper();
        //     void Callback(const sensor_msgs::PointCloud2& msg);

        // private:
        //     // Define any private members or helper functions here
    // };

    class Cropper {
        public:
            Cropper(const ros::Publisher& pub);
            void Callback(const sensor_msgs::PointCloud2& msg);

        private:
            ros::Publisher pub_;
    };
} // namespace perception

#endif // PERCEPTION_CROP_H
