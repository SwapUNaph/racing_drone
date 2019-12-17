#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include "racing_drone/GateDetector.hpp"

class GateDetectorCore
{
    public:
        ros::NodeHandle nh;
        ros::Publisher gatePosePub;
        std::string pubTopic;

        GateDetector gd;
        geometry_msgs::Pose gatePose; 

        GateDetectorCore(ros::NodeHandle &node_handle, GateDetector gd_, std::string pubTopic_);
        ~GateDetectorCore();

        void publishGatePose(Mat& img); 

};