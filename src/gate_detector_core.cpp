/**
 * @file gate_detector_core.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief gate_detector_core definition
 * @version 0.1
 * @date 01-05-2020
 * 
 *  Copyright (c) 2020 Swapneel Naphade
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */
#include "racing_drone/gate_detector_core.hpp"
#include "common.cpp"

/**
 * @brief Construct a new Gate Detector Core:: Gate Detector Core object
 * 
 * @param node_handle ROS Node handle 
 * @param gd_ GateDetector object
 * @param pubTopic_ Gate pose publication topic
 */
GateDetectorCore::GateDetectorCore(ros::NodeHandle &node_handle, GateDetector gd_, ExtendedKalmanFilter gateEKF_,
    std::string rawGatePubTopic_, std::string filteredGatePubTopic_, std::string odomSubTopic_, std::string imageTopic_) 
                            :   nh(node_handle) , gd(gd_), gateEKF(gateEKF_), rawGatePubTopic(rawGatePubTopic_),
                                 filteredGatePubTopic(filteredGatePubTopic_), odomSubTopic(odomSubTopic_), imageTopic(imageTopic_)
{
    rawGatePosePub = nh.advertise<geometry_msgs::Pose>(rawGatePubTopic, 5);
    filteredGatePosePub = nh.advertise<geometry_msgs::Pose>(filteredGatePubTopic, 5);

    odomSub = nh.subscribe(odomSubTopic, 10, &GateDetectorCore::odomCallback, this);
    imageSub = nh.subscribe(imageTopic, 2, &GateDetectorCore::imageCallback, this);

    uEKF.resize(3);
    wEKF.resize(3);
    yEKF.resize(3);

    rawGatePose = geometry_msgs::Pose();
    filteredGatePose = geometry_msgs::Pose();

}


/**
 * @brief Destroy the Gate Detector Core:: Gate Detector Core object
 * 
 */
GateDetectorCore::~GateDetectorCore(){}

void GateDetectorCore::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    // ROS_INFO( "In odom Callback." );
    uEKF(0) = odom->twist.twist.linear.x;
    uEKF(1) = odom->twist.twist.linear.y;
    uEKF(2) = odom->twist.twist.linear.z;

    ublas::matrix<double> bRo(3,3);
    std::vector<double> quat(4);
    quat[0] = odom->pose.pose.orientation.x;
    quat[1] = odom->pose.pose.orientation.y;
    quat[2] = odom->pose.pose.orientation.z;
    quat[3] = odom->pose.pose.orientation.w;
    
    uEKF = rotateVec(quat, uEKF); //Velocity in body frame

    wEKF[0] = odom->twist.twist.angular.x;
    wEKF[1] = odom->twist.twist.angular.y;
    wEKF[2] = odom->twist.twist.angular.z;
}

/**
 * @brief Detect Gate, Calcualte Gate Pose and publish Gate Pose
 * 
 * @param img Input image for gate detection
 */
void GateDetectorCore::publishGatePose(Mat& img)
{
    // ROS_INFO( " before detectgate ");
    bool GATE_DETECTION_SUCCESS = gd.detectGate(img);
    bool GATE_POSE_SUCCESS = false;

    // ROS_INFO( " before get gate pose ");
    if( GATE_DETECTION_SUCCESS)
        GATE_POSE_SUCCESS = gd.getGatePose();

    if( GATE_DETECTION_SUCCESS &&  GATE_POSE_SUCCESS )
    {
        rawGatePose.position.x = gd.tvec[0];
        rawGatePose.position.y = gd.tvec[1];
        rawGatePose.position.z = gd.tvec[2];
        std::vector<double> gateQ(rvec2quat(gd.rvec));
        rawGatePose.orientation.x = gateQ[0];
        rawGatePose.orientation.y = gateQ[1];
        rawGatePose.orientation.z = gateQ[2];
        rawGatePose.orientation.w = gateQ[3];

        yEKF(0) = gd.tvec[0];
        yEKF(1) = gd.tvec[1];
        yEKF(2) = gd.tvec[2];
        gateEKF.filter(wEKF, uEKF, yEKF);
        filteredGatePose.position.x = gateEKF.X(0);
        filteredGatePose.position.y = gateEKF.X(1);
        filteredGatePose.position.z = gateEKF.X(2);
        filteredGatePose.orientation.x = gateQ[0];
        filteredGatePose.orientation.y = gateQ[1];
        filteredGatePose.orientation.z = gateQ[2];
        filteredGatePose.orientation.w = gateQ[3];

        rawGatePosePub.publish(rawGatePose);
    }
    else
    {
        gateEKF.predict(wEKF, uEKF);
        filteredGatePose.position.x = gateEKF.X(0);
        filteredGatePose.position.y = gateEKF.X(1);
        filteredGatePose.position.z = gateEKF.X(2);   
    }
    
    filteredGatePosePub.publish(filteredGatePose);
    
}

void GateDetectorCore::imageCallback(const sensor_msgs::Image::ConstPtr& ros_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);
    // ros::Time begin = ros::Time::now();
    publishGatePose(cv_ptr->image);
    // ros::Time end = ros::Time::now();
    // double loopTime = end.toNSec() - begin.toNSec();
    // ROS_INFO("Gate detector loop time: %f ms\n", loopTime/1e6);
}


