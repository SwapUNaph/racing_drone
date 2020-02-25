/**
 * @file localizer_core.hpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief Localizer core declaration
 * @version 0.1
 * @date 01-15-2020
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

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <racing_drone/DroneState.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "KalmanFilter.hpp"


class Localizer
{
    public:
    ros::NodeHandle nh;
    ros::Publisher odomPublisher;
    ros::Subscriber odomSubscriber;
    ros::Subscriber gatePoseSubscriber;
    ros::Subscriber gateDetectionSuccess;
    std::string odomSubTopic;
    std::string odomPubTopic;
    std::string gatePoseTopic;
    double measGain;
    double actualGain;
    bool gdSuccess;
    nav_msgs::Odometry inOdom;
    nav_msgs::Odometry outOdom;
    geometry_msgs::Pose gatePoseDrone;
    geometry_msgs::Pose gatePoseOrigin;

    std::vector<racing_drone::DroneState> gates;
    int visualGateIndex;

    Localizer(ros::NodeHandle& nh_, std::string odomSubTopic_, std::string odomPubTopic_, std::string gatePoseTopic_,
              std::vector<racing_drone::DroneState> gates_, double measGain_);
    ~Localizer();

    void odomSubCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void gatePoseSubCallback(const geometry_msgs::Pose::ConstPtr& gtPose);
    void gdSuccessCallback(const std_msgs::Bool::ConstPtr& succ);
    void publishDroneState(void);
    void findGate(void);
    double getGateDistance(racing_drone::DroneState gateState);
    void projectGatePose(void);

};

class GateVisual
{
    public:
    GateVisual(ros::NodeHandle& nh_, std::vector<racing_drone::DroneState> gates_);
    ros::NodeHandle nh;
    ros::Publisher gateMarkerPublisher;
    ros::Timer markerTimer;
    visualization_msgs::MarkerArray gateMarkers;
    std::vector<racing_drone::DroneState> gates;
    void publishGateMarkersCallback(const ros::TimerEvent& timerEvent);
};