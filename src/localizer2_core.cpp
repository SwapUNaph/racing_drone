/**
 * @file localizer2_core.cpp
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


#include <racing_drone/localizer2_core.hpp>
#include "common.cpp"

Localizer::Localizer(ros::NodeHandle& nh_, std::string odomSubTopic_, std::string odomPubTopic_, std::string gatePoseTopic_,
              std::vector<racing_drone::DroneState> gates_, double pr, std::vector<double> q, std::vector<double> r) :
                nh(nh_), odomSubTopic(odomSubTopic_), odomPubTopic(odomPubTopic_), gatePoseTopic(gatePoseTopic_), gates(gates_), predict_rate(pr), rawGateMVA(3, 3)
{
    odomPublisher = nh.advertise<nav_msgs::Odometry>(odomPubTopic, 5);
    odomSubscriber = nh.subscribe(odomSubTopic, 5, &Localizer::odomSubCallback, this);
    gatePoseSubscriber = nh.subscribe(gatePoseTopic, 5, &Localizer::gatePoseSubCallback, this);
    predictorTimer = nh.createTimer(ros::Duration(1.0/predict_rate), &Localizer::predictorTimerCallback, this);
    publishTimer = nh.createTimer(ros::Duration(1.0/predict_rate), &Localizer::publishTimerCallback, this);
    gateDetectionSuccess = nh.subscribe("/gate_detector/success", 5, &Localizer::gdSuccessCallback, this);

    inOdom = nav_msgs::Odometry();
    outOdom = inOdom;
    droneQ.resize(4);
    droneR.resize(3,3);
    Q.resize(3,3);
    R.resize(3,3);
    P.resize(3,3);
    X.resize(3);
    X_gt.resize(3);
    D_r_GD.resize(3);
    I_r_GD.resize(3);
    I_r_GO.resize(3);
    body_vel.resize(3);
    // Identity
    I.resize(3,3);

    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
        {
            if (i==j)
            {
                Q(i,j) = q[i];
                P(i,j) = q[i];
                R(i,j) = r[i];
                I(i,j) = 1.0;
            }
            else
            {
                Q(i,j) = 0.0;
                P(i,j) = 0.0;
                R(i,j) = 0.0;
                I(i,j) = 0.0;
            }
            X(i) = 0.0;
        }

    droneR = I;
    D_r_GD = X;
    I_r_GD = X;
    I_r_GO = X;
    body_vel = X;
    X_gt = X;
    // gatePoseOrigin.position = gates[0].position;


    visualGateIndex = 0;
    I_r_GO(0) = gates[visualGateIndex].position.x;
    I_r_GO(1) = gates[visualGateIndex].position.y;
    I_r_GO(2) = gates[visualGateIndex].position.z;

    gate_pose_count = 0;

    ROS_INFO( "Localizer Initialized ...");
}

void Localizer::odomSubCallback(const nav_msgs::Odometry::ConstPtr& odomIn)
{
    inOdom = *odomIn;
    outOdom = inOdom;

    X_gt(0) = inOdom.pose.pose.position.x;
    X_gt(1) = inOdom.pose.pose.position.y;
    X_gt(2) = inOdom.pose.pose.position.z;

    body_vel(0) = inOdom.twist.twist.linear.x;
    body_vel(1) = inOdom.twist.twist.linear.y;
    body_vel(2) = inOdom.twist.twist.linear.z; // Inertial velocity if simulation, body velocity if bebop

    droneQ[0] = inOdom.pose.pose.orientation.x;
    droneQ[1] = inOdom.pose.pose.orientation.y;
    droneQ[2] = inOdom.pose.pose.orientation.z;
    droneQ[3] = inOdom.pose.pose.orientation.w;

    quat2rotmat(droneQ, droneR);

    body_vel = prod( trans(droneR), body_vel); // Only in sim., commment while using bebop

    // ROS_INFO("Localizer odom received.");
    // std::cout << "R: " << droneR << std::endl;
}

void Localizer::predictorTimerCallback(const ros::TimerEvent& timerEvent)
{
    double dt = 1.0 / predict_rate;
    double dt2 = dt*dt;

    ublas::matrix<double> QRt( prod( Q, trans(droneR) ));
    // state_mutex.lock();
    X = X  +  prod( droneR, body_vel) * dt;
    P = P  +  prod( droneR, QRt ) * dt2;
    // state_mutex.unlock();

    // ROS_INFO("Localizer predicted next step.");
    // std::cout << X << std::endl;
    // std::cout << P << std::endl;
}

void Localizer::publishTimerCallback(const ros::TimerEvent& timerEvent)
{

    outOdom.pose.pose.position.x = X(0);
    outOdom.pose.pose.position.y = X(1);
    outOdom.pose.pose.position.z = X(2);
    outOdom.pose.covariance[0] = P(0,0);
    outOdom.pose.covariance[4] = P(1,1);
    outOdom.pose.covariance[8] = P(2,2);

    odomPublisher.publish(outOdom);
    // ROS_INFO("Localizer odom published.");
    // std::cout << X << std::endl;
    // std::cout << P << std::endl;
}


void Localizer::gatePoseSubCallback(const geometry_msgs::Pose::ConstPtr& gtPose) 
{
    if( gate_pose_count++ > 3)
    {
        D_r_GD(0) = gtPose->position.x;
        D_r_GD(1) = gtPose->position.y;
        D_r_GD(2) = gtPose->position.z;

        // D_r_GD = rawGateMVA.update(D_r_GD); // Filter raw gate measurement
        std::cout << D_r_GD << std::endl;

        projectGatePose();
        findGate();
        updateDroneState();

        // ROS_INFO( "Localizer raw gate pose callback.");
        // std::cout << X << std::endl;
        // std::cout << P << std::endl;
    }
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

    ROS_INFO( "[localizer] current gate index: %d", visualGateIndex );

    // Update actual gate position
    I_r_GO(0) = gates[visualGateIndex].position.x;
    I_r_GO(1) = gates[visualGateIndex].position.y;
    I_r_GO(2) = gates[visualGateIndex].position.z;
}

/**
 * @brief Projects measured gate pose in drone frame to inertial frame using drone's current pose
 *        { r_go = r_do + o_R_d * r_gd ; q_go = q_gd (x) q_do}
 * 
 */
void Localizer::projectGatePose(void)
{
    // Project gate position in inertial frame
    I_r_GO = X_gt + prod( droneR, D_r_GD);
    std::cout << I_r_GO << std::endl;
}

double Localizer::getGateDistance(racing_drone::DroneState gateState)
{
    return std::sqrt( (gateState.position.x - I_r_GO(0))*(gateState.position.x - I_r_GO(0)) +
                      (gateState.position.y - I_r_GO(1))*(gateState.position.y - I_r_GO(1)) +
                      (gateState.position.z - I_r_GO(2))*(gateState.position.z - I_r_GO(2)) );
}

void Localizer::updateDroneState(void)
{
    ublas::matrix<double> H(-1.0 * trans(droneR)); // H = -iRd'
    ublas::matrix<double> PHt(prod(P,trans(H)));
    ublas::matrix<double> S( prod(H, PHt) + R ); // HPH' + R, H = -iRd'

    // S_inv
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
        {
            if(i==j)
                S(i,j) = 1.0/S(i,j);
            else
                S(i,j) = 0.0;
        }


    ublas::matrix<double> K( prod( PHt, S) ); // PHt(S^-1)
    ublas::matrix<double> IKH( I - prod(K, H) );
    ublas::matrix<double> P_IKH_t( prod(P, trans(IKH)) );
    ublas::matrix<double> RKt( prod(R, trans(K)) );
    ublas::vector<double> d_r_gd( prod( trans(droneR), (I_r_GO - X) ));

    // state_mutex.lock();
    X = X + prod( K, ( D_r_GD - d_r_gd ) ); // X + K (D_r_GO - iRd'(I_r_GO - X))
    P = prod(IKH, P_IKH_t) + prod(K, RKt);
    // state_mutex.unlock();

}

void Localizer::gdSuccessCallback(const std_msgs::Bool::ConstPtr& succ)
{
    gdSuccess = succ->data;
    if(!gdSuccess)
        gate_pose_count = 0;
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