/**
 * @file localizer_core.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief localizer core definition
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


#include <racing_drone/localizer_core.hpp>
#include "common.cpp"

Localizer::Localizer(ros::NodeHandle& nh_, std::string odomSubTopic_, std::string odomPubTopic_, std::string gatePoseTopic_,
              std::vector<racing_drone::DroneState> gates_, double measGain_)
              : nh(nh_), odomSubTopic(odomSubTopic_), odomPubTopic(odomPubTopic_),
                gatePoseTopic(gatePoseTopic_), gates(gates_), measGain(measGain_)
{
    odomPublisher = nh.advertise<nav_msgs::Odometry>(odomPubTopic, 5);
    actGatePosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/localizer/actual_gate_pose", 5);
    odomSubscriber = nh.subscribe(odomSubTopic, 5, &Localizer::odomSubCallback, this);
    gatePoseSubscriber = nh.subscribe(gatePoseTopic, 5, &Localizer::gatePoseSubCallback, this);
    gateDetectionSuccess = nh.subscribe("/gate_detector/success", 5, &Localizer::gdSuccessCallback, this);

    inOdom = nav_msgs::Odometry();
    outOdom = inOdom;
    gatePoseDrone = geometry_msgs::Pose();
    gatePoseOrigin = geometry_msgs::Pose();
    actualGatePoseDrone = geometry_msgs::PoseStamped();

    // gatePoseOrigin.position = gates[0].position;

    visualGateIndex = 0;

    droneR.resize(3,3);
    droneQ.resize(4);
    PosCov.resize(3,3);

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
        {
            if(i==j) droneR(i,j) = 1.0;
            else droneR(i,j) = 0.0;
        }


    ROS_INFO( "Localizer Initialized ...");
}

void Localizer::odomSubCallback(const nav_msgs::Odometry::ConstPtr& odomIn)
{
    inOdom = *odomIn;
    droneQ[0] = inOdom.pose.pose.orientation.x;
    droneQ[1] = inOdom.pose.pose.orientation.y;
    droneQ[2] = inOdom.pose.pose.orientation.z;
    droneQ[3] = inOdom.pose.pose.orientation.w;

    // ROS_INFO( "Localizer Odom In.");
}

void Localizer::gatePoseSubCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& gtPoseSt) 
{
    // ROS_INFO( "Localizer gate pose callback.");
    gatePoseDrone = gtPoseSt->pose.pose;
    
    for(int i=0; i<9; i++)
        PosCov(i/3,i%3) = gtPoseSt->pose.covariance[i];

    // gatePoseDrone.position = oRd * gatePose.position;

    std::vector<double> gatePosDrone(3);

    quat2rotmat(droneQ, droneR);

    gatePosDrone[0] = gatePoseDrone.position.x;
    gatePosDrone[1] = gatePoseDrone.position.y;
    gatePosDrone[2] = gatePoseDrone.position.z;

    gatePosDrone = rotateVec(droneQ, gatePosDrone);

    gatePoseDrone.position.x = gatePosDrone[0];
    gatePoseDrone.position.y = gatePosDrone[1];
    gatePoseDrone.position.z = gatePosDrone[2]; // Converted to gate position wrt drone in inertial frame from drone frame.


    projectGatePose();
    findGate();
    publishDroneState();
}

Localizer::~Localizer(){}

void Localizer::findGate(void)
{
    int minDistIndex = 0;
    double minDist = 10000;

    for(int i=0; i < gates.size(); i++)
    {
        double dist = getGateDistance(gates[i]);
        if( dist < minDist)
        {
            minDist = dist;
            minDistIndex = i;
        }
    }

    visualGateIndex = minDistIndex;

    // ROS_INFO( "[localizer] current gate index: %d", visualGateIndex );

    std::vector<double> gateRPY(3), gateQ(4);
    gatePoseOrigin.position.x = gates[visualGateIndex].position.x;
    gatePoseOrigin.position.y = gates[visualGateIndex].position.y;
    gatePoseOrigin.position.z = gates[visualGateIndex].position.z;

    gateRPY[0] = 0.0;
    gateRPY[1] = 0.0;
    gateRPY[2] = gates[visualGateIndex].yaw;

    rpy2quat(gateRPY, gateQ);

    gatePoseOrigin.orientation.x = gateQ[0];
    gatePoseOrigin.orientation.y = gateQ[1];
    gatePoseOrigin.orientation.z = gateQ[2];
    gatePoseOrigin.orientation.w = gateQ[3];

    //IrGD = IrGO - IrDO
    //DrGD = dRi * (IrGO - IrDO)
    std::vector<double> IrGD(3), DrGD(3);
    IrGD[0] = gatePoseOrigin.position.x - inOdom.pose.pose.position.x;
    IrGD[1] = gatePoseOrigin.position.y - inOdom.pose.pose.position.y;
    IrGD[2] = gatePoseOrigin.position.z - inOdom.pose.pose.position.z;

    DrGD = rotateVec(quatConjugate(droneQ), IrGD);

    actualGatePoseDrone.pose.position.x = DrGD[0];
    actualGatePoseDrone.pose.position.y = DrGD[1];
    actualGatePoseDrone.pose.position.z = DrGD[2];
    actualGatePoseDrone.header.stamp = ros::Time::now();
    actGatePosePublisher.publish(actualGatePoseDrone);
}

/**
 * @brief Projects measured gate pose in drone frame to inertial frame using drone's current pose
 *        { r_go = r_do + o_R_d * r_gd ; q_go = q_gd (x) q_do}
 * 
 */
void Localizer::projectGatePose(void)
{
    // gate = odomIn.pose.pose.position + oRd * gatePose.position;
    std::vector<double> gateQDrone(4), gateQOrigin(4), droneQ(4);
    droneQ[0] = inOdom.pose.pose.orientation.x;
    droneQ[1] = inOdom.pose.pose.orientation.y;
    droneQ[2] = inOdom.pose.pose.orientation.z;
    droneQ[3] = inOdom.pose.pose.orientation.w;

    gateQDrone[0] = gatePoseDrone.orientation.x;
    gateQDrone[1] = gatePoseDrone.orientation.y;
    gateQDrone[2] = gatePoseDrone.orientation.z;
    gateQDrone[3] = gatePoseDrone.orientation.w;

    gateQOrigin = quatMultiply(gateQDrone, droneQ);

    gatePoseOrigin.position.x = inOdom.pose.pose.position.x + gatePoseDrone.position.x;
    gatePoseOrigin.position.y = inOdom.pose.pose.position.y + gatePoseDrone.position.y;
    gatePoseOrigin.position.z = inOdom.pose.pose.position.z + gatePoseDrone.position.z;
    gatePoseOrigin.orientation.x = gateQOrigin[0];
    gatePoseOrigin.orientation.y = gateQOrigin[1];
    gatePoseOrigin.orientation.z = gateQOrigin[2];
    gatePoseOrigin.orientation.w = gateQOrigin[3];

}

double Localizer::getGateDistance(racing_drone::DroneState gateState)
{
    return std::sqrt( (gateState.position.x - gatePoseOrigin.position.x)*(gateState.position.x - gatePoseOrigin.position.x) +
                      (gateState.position.y - gatePoseOrigin.position.y)*(gateState.position.y - gatePoseOrigin.position.y) +
                      (gateState.position.z - gatePoseOrigin.position.z)*(gateState.position.z - gatePoseOrigin.position.z) );
}

void Localizer::publishDroneState(void)
{
    outOdom = nav_msgs::Odometry();
    outOdom.header.frame_id = "map";
    outOdom.pose.pose.position.x = inOdom.pose.pose.position.x + measGain * (gatePoseOrigin.position.x - gatePoseDrone.position.x - inOdom.pose.pose.position.x);
    outOdom.pose.pose.position.y = inOdom.pose.pose.position.y + measGain * (gatePoseOrigin.position.y - gatePoseDrone.position.y - inOdom.pose.pose.position.y);
    outOdom.pose.pose.position.z = inOdom.pose.pose.position.z + measGain * (gatePoseOrigin.position.z - gatePoseDrone.position.z - inOdom.pose.pose.position.z);

    // std::vector<double> gatePoseOriginQ(4), gateRPY(3), droneQ(4), droneRPY(3), gateWRTDroneQ(4), gateWRTDroneRPY(3);
    // gatePoseOriginQ[0] = gatePoseOrigin.orientation.x;
    // gatePoseOriginQ[1] = gatePoseOrigin.orientation.y;
    // gatePoseOriginQ[2] = gatePoseOrigin.orientation.z;
    // gatePoseOriginQ[3] = gatePoseOrigin.orientation.w;

    // droneQ[0] = inOdom.pose.pose.orientation.x;
    // droneQ[1] = inOdom.pose.pose.orientation.y;
    // droneQ[2] = inOdom.pose.pose.orientation.z;
    // droneQ[3] = inOdom.pose.pose.orientation.w;

    // gateWRTDroneQ[0] = gatePoseDrone.orientation.x;
    // gateWRTDroneQ[1] = gatePoseDrone.orientation.y;
    // gateWRTDroneQ[2] = gatePoseDrone.orientation.z;
    // gateWRTDroneQ[3] = gatePoseDrone.orientation.w;

    // quat2rpy(gatePoseOriginQ, gateRPY);
    // quat2rpy(droneQ, droneRPY);
    // quat2rpy(gateWRTDroneQ, gateWRTDroneRPY);

    // droneRPY[2] = droneRPY[2] + actualGain * (gateRPY[2] - gateWRTDroneQ[2] - droneRPY[2]);

    // rpy2quat(droneRPY, droneQ);

    ublas::matrix<double> PR( prod(PosCov, droneR));
    PosCov = prod(trans(droneR), PR);

    for(int i=0; i<9; i++)
        outOdom.pose.covariance[i] = PosCov(i/3,i%3); 

    outOdom.pose.pose.orientation = inOdom.pose.pose.orientation;
    outOdom.header.stamp = ros::Time::now();

    odomPublisher.publish(outOdom);
}

void Localizer::gdSuccessCallback(const std_msgs::Bool::ConstPtr& succ)
{
    gdSuccess = succ->data;
}

/////////////////////////// GATE VISUALS ///////////////////////////////////

GateVisual::GateVisual(ros::NodeHandle& nh_, std::vector<racing_drone::DroneState> gates_) : nh(nh_), gates(gates_)
{
    gateMarkerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/gate_marker_array", 5);
    markerTimer = nh.createTimer(ros::Duration(5), &GateVisual::publishGateMarkersCallback, this);
    visualization_msgs::Marker baseMarker;
    baseMarker.type  = visualization_msgs::Marker::CUBE;
    baseMarker.header.frame_id = "map";
    baseMarker.color.r = 255.0f;
    baseMarker.color.g = 50.0f;
    baseMarker.color.b = 0.0f;
    baseMarker.color.a = 0.50;  
    baseMarker.scale.x = 0.20;
    baseMarker.scale.y = 1.0;
    baseMarker.scale.z = 1.0;
    baseMarker.action = visualization_msgs::Marker::ADD;
    baseMarker.lifetime = ros::Duration();


    for(int i=0; i<gates.size(); i++)
    {
        visualization_msgs::Marker gateMarker(baseMarker);
        gateMarker.id = i;
        gateMarker.pose.position.x = gates[i].position.x;
        gateMarker.pose.position.y = gates[i].position.y;
        gateMarker.pose.position.z = gates[i].position.z;

        std::vector<double> gateQuat(4), gateRPY(3);
        gateRPY[0] = 0.0;
        gateRPY[1] = 0.0;
        gateRPY[2] = gates[i].yaw;

        rpy2quat(gateRPY, gateQuat);

        gateMarker.pose.orientation.x = gateQuat[0];
        gateMarker.pose.orientation.y = gateQuat[1];
        gateMarker.pose.orientation.z = gateQuat[2];
        gateMarker.pose.orientation.w = gateQuat[3];

        gateMarkers.markers.push_back(gateMarker);
    }

}

void GateVisual::publishGateMarkersCallback(const ros::TimerEvent& timerEvent)
{
    gateMarkerPublisher.publish(gateMarkers);
}