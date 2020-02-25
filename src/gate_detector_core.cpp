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
             std::string rawGatePubTopic_, std::string filteredGatePubTopic_, std::string odomSubTopic_,
             std::string imageOutTopic_, std::string imuTopic_) 
                            :   nh(node_handle) , gd(gd_), gateEKF(gateEKF_), gateMAF(3,3), rawGatePubTopic(rawGatePubTopic_),
                                 filteredGatePubTopic(filteredGatePubTopic_), odomSubTopic(odomSubTopic_),
                                 imageOutTopic(imageOutTopic_), imuTopic(imuTopic_)
{
    rawGatePosePub = nh.advertise<geometry_msgs::Pose>(rawGatePubTopic, 5);
    filteredGatePosePub = nh.advertise<geometry_msgs::Pose>(filteredGatePubTopic, 5);
    successPub = nh.advertise<std_msgs::Bool>("/gate_detector/success", 5);
    gateImagePub = nh.advertise<sensor_msgs::Image>("/gate_detector/gate_image", 2);

    odomSub = nh.subscribe(odomSubTopic, 5, &GateDetectorCore::odomCallback, this);
    imageSub = nh.subscribe(imageOutTopic, 2, &GateDetectorCore::imageCallback, this);
    imuSub = nh.subscribe(imuTopic, 2, &GateDetectorCore::imuCallback, this);
    EKF_timer = nh.createTimer(ros::Duration(gateEKF.dt), &GateDetectorCore::EKF_timerCallback, this);

    uEKF.resize(3);
    wEKF.resize(3);
    yEKF.resize(3);

    uEKF(0) = 0.0;
    uEKF(1) = 0.0;
    uEKF(2) = 0.0;

    yEKF(0) = 0.0;
    yEKF(1) = 0.0;
    yEKF(2) = 0.0;
    // yEKF(3) = 0.0;
    // yEKF(4) = 0.0;
    // yEKF(5) = 0.0;

    droneQ.resize(4);
    rawGatePose = geometry_msgs::Pose();
    filteredGatePose = geometry_msgs::Pose();
    success = std_msgs::Bool();

    // Setup Markers
    initMarkers();

    ros::Time nowTime = ros::Time::now();
    srand(nowTime.toNSec());
}


/**
 * @brief Destroy the Gate Detector Core:: Gate Detector Core object
 * 
 */
GateDetectorCore::~GateDetectorCore(){}

void GateDetectorCore::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    wEKF[0] = odom->twist.twist.angular.x;
    wEKF[1] = odom->twist.twist.angular.y;
    wEKF[2] = odom->twist.twist.angular.z;

    // ROS_INFO( "In odom Callback." );
    ublas::vector<double> body_vel(3);
    body_vel(0) = odom->twist.twist.linear.x;
    body_vel(1) = odom->twist.twist.linear.y;
    body_vel(2) = odom->twist.twist.linear.z; // Velocity in inertial frame

    droneQ[0] = odom->pose.pose.orientation.x;
    droneQ[1] = odom->pose.pose.orientation.y;
    droneQ[2] = odom->pose.pose.orientation.z;
    droneQ[3] = odom->pose.pose.orientation.w;
    
    uEKF = rotateVec(quatConjugate(droneQ), body_vel); // Velocity in body frame
    wEKF = rotateVec(quatConjugate(droneQ), wEKF); // Angular velocity in body frame

    // uEKF(0) = body_vel(0);
    // uEKF(1) = body_vel(1);
    // uEKF(2) = body_vel(2);  // Measure drone velocity in body frame
}

/**
 * @brief Detect Gate, Calcualte Gate Pose and publish Gate Pose
 * 
 * @param img Input image for gate detection
 */
void GateDetectorCore::updateGatePose(Mat& img)
{
    Mat imgCopy;
    img.copyTo(imgCopy);
    // ROS_INFO( " before detectgate ");
    bool GATE_DETECTION_SUCCESS = gd.detectGate(img);
    bool GATE_POSE_SUCCESS = false;


    // ROS_INFO( " before get gate pose ");
    if( GATE_DETECTION_SUCCESS)
        GATE_POSE_SUCCESS = gd.getGatePose();


    if( GATE_DETECTION_SUCCESS &&  GATE_POSE_SUCCESS )
    {

        // Generate noise to simulate actual gate detection (0.0 < random < 1.0)
        double noisex = (rand() % 10000) / 10000.0;
        double noisey = (rand() % 10000) / 10000.0;
        double noisez = (rand() % 10000) / 10000.0;


        yEKF(0) = gd.tvec[0] + noisex / 10.0;
        yEKF(1) = gd.tvec[1] + noisey / 10.0;
        yEKF(2) = gd.tvec[2] + noisez / 10.0;

        rawGatePose.position.x = yEKF(0);
        rawGatePose.position.y = yEKF(1);
        rawGatePose.position.z = yEKF(2);
        std::vector<double> gateQ(rvec2quat(gd.rvec));
        rawGatePose.orientation.x = gateQ[0];
        rawGatePose.orientation.y = gateQ[1];
        rawGatePose.orientation.z = gateQ[2];
        rawGatePose.orientation.w = gateQ[3];


        //Update EKF
        yEKF = gateMAF.update(yEKF);
        gateEKF.update(wEKF, yEKF);
        filteredGatePose.position.x = gateEKF.X(0);
        filteredGatePose.position.y = gateEKF.X(1);
        filteredGatePose.position.z = gateEKF.X(2);
        filteredGatePose.orientation.x = gateQ[0];
        filteredGatePose.orientation.y = gateQ[1];
        filteredGatePose.orientation.z = gateQ[2];
        filteredGatePose.orientation.w = gateQ[3];

        // Publish raw gate pose and raw gate pose marker
        rawGatePosePub.publish(rawGatePose);
        rawGateMarker.pose = rawGatePose;
        rawGateMarker.header.stamp = ros::Time::now();
        rawMarkerPub.publish(rawGateMarker);

        success.data = true;
        drawContours(imgCopy, gd.detectedContours, -1, Scalar(0,255,0), 2, 8);
        drawContours(imgCopy, std::vector<std::vector<Point>>(1,gd.gateContour), -1, Scalar(255,0,0), 4, 8);

    }
    else
    {
        success.data = false;  
    }
    
    successPub.publish(success);

    // Publish the image with/without gate 
    cv_bridge::CvImage cvImg;
    cvImg.encoding = "bgr8";
    cvImg.image = imgCopy;
    gateImagePub.publish(cvImg.toImageMsg());
    
}

void GateDetectorCore::imageCallback(const sensor_msgs::Image::ConstPtr& ros_img)
{
    cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);
    updateGatePose(cv_ptr->image);
}

void GateDetectorCore::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{

    // ublas::vector<double> gravity(3);
    // gravity(0) = 0.0;
    // gravity(1) = 0.0;
    // gravity(2) = -9.8; // gravity in inertial frame

    // gravity = rotateVec( quatConjugate(droneQ), gravity); // gravity in body frame

    // uEKF(0) = imu->linear_acceleration.x + gravity(0);
    // uEKF(1) = imu->linear_acceleration.y + gravity(1);
    // uEKF(2) = imu->linear_acceleration.z + gravity(2);

}

void GateDetectorCore::EKF_timerCallback(const ros::TimerEvent& timerEvent)
{
    // Predict gate pose
    gateEKF.predict(wEKF, uEKF);
    filteredGatePose.position.x = gateEKF.X(0);
    filteredGatePose.position.y = gateEKF.X(1);
    filteredGatePose.position.z = gateEKF.X(2);

    // Publish filtered gate pose and marker
    filteredGatePosePub.publish(filteredGatePose);
    filtGateMarker.pose = filteredGatePose;
    filtGateMarker.header.stamp = ros::Time::now();
    filtMarkerPub.publish(filtGateMarker);
}

void GateDetectorCore::initMarkers(void)
{
    rawMarkerPub = nh.advertise<visualization_msgs::Marker>("/raw_gate_marker", 5);
    filtMarkerPub = nh.advertise<visualization_msgs::Marker>("/filtered_gate_marker", 5);

    rawGateMarker = visualization_msgs::Marker();
    filtGateMarker = visualization_msgs::Marker();
    rawGateMarker.id = 0;
    filtGateMarker.id = 1;
    rawGateMarker.type  = visualization_msgs::Marker::SPHERE;
    filtGateMarker.type = visualization_msgs::Marker::SPHERE;
    rawGateMarker.header.frame_id = "base_link";
    filtGateMarker.header.frame_id = "base_link";
    rawGateMarker.color.r = 0.0f;
    rawGateMarker.color.g = 255.0f;
    rawGateMarker.color.b = 0.0f;
    rawGateMarker.color.a = 0.75;  
    filtGateMarker.color.r = 255.0f;
    filtGateMarker.color.g = 0.0f;
    filtGateMarker.color.b = 0.0f;
    filtGateMarker.color.a = 1.0; 
    rawGateMarker.scale.x = 0.20;
    rawGateMarker.scale.y = 0.20;
    rawGateMarker.scale.z = 0.20;
    filtGateMarker.scale.x = 0.20;
    filtGateMarker.scale.y = 0.20;
    filtGateMarker.scale.z = 0.20;
    rawGateMarker.action = visualization_msgs::Marker::ADD;
    filtGateMarker.action = visualization_msgs::Marker::ADD;
    rawGateMarker.lifetime = ros::Duration(0.1);
    filtGateMarker.lifetime = ros::Duration(0.1);
}


