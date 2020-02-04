/**
 * @file state_estimator_core.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief state_estimator definition
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

#include "racing_drone/state_estimator_core.hpp"

/**
 * @brief Construct a new State Estimator:: State Estimator object
 * 
 * @param node_handle ROS Node handle
 * @param pub_topic Publication Topic of odometry_out
 * @param sub_topic Subscription Topic of odometry_in
 * @param rt State Estimator Rate
 * @param kf Kalman Filter object
 */
StateEstimator::StateEstimator(const ros::NodeHandle &node_handle,
							const std::string& pub_topic,
							const std::string& sub_topic,
							int rt, KalmanFilter kf)
							 : nh(node_handle), pubTopic(pub_topic), subTopic(sub_topic),
							 rate(rt), KF(kf)
{
	this->init();
}

/**
 * @brief Initialize estimator
 * 
 */
void StateEstimator::init()
{
	odomSubscriber = nh.subscribe(subTopic, 5, &StateEstimator::odomCallback, this);
	odomPublisher = nh.advertise<nav_msgs::Odometry>(pubTopic, 5);
	estimatorLoopTimer = nh.createTimer(ros::Duration(1.0/rate),  &StateEstimator::estimatorLoopTimerCallback, this);
}

/**
 * @brief Destroy the State Estimator:: State Estimator object
 * 
 */
StateEstimator::~StateEstimator()
{

}

/**
 * @brief Timer Callback for publishing state estimate
 * 
 * @param timerEvent 
 */
void StateEstimator::estimatorLoopTimerCallback(const ros::TimerEvent& timerEvent)
{
	publishOdometry();
	ros::Rate rate(this->rate);
	rate.sleep();
}

/**
 * @brief Callback for odometry input
 * 
 * @param odom 
 */
void StateEstimator::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	odomOut = *odom;
	vector<double> y(3);
	y(0) = odom->pose.pose.position.x;
	y(1) = odom->pose.pose.position.y;
	y(2) = odom->pose.pose.position.z;
	
	
	//Examples
	// ROS_INFO("Seq: [%d]", odom->header.seq);
	// ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odom->pose.pose.position.x,odom->pose.pose.position.y, odom->pose.pose.position.z);
	// ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
	// ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", odom->twist.twist.linear.x,odom->twist.twist.angular.z); 
	
	//Update the estimate
	KF.update(y);
	odomOut.pose.pose.position.x = KF.X(0);
	odomOut.pose.pose.position.y = KF.X(1);
	odomOut.pose.pose.position.z = KF.X(2);
	// ROS_INFO("[StateEstimator] Odom recieved.");
}

/**
 * @brief Publish Odometry after filtering
 * 
 */
void StateEstimator::publishOdometry(void)
{
	vector<double> u(3);
	u(0) = odomOut.twist.twist.linear.x;
	u(1) = odomOut.twist.twist.linear.y;
	u(2) = odomOut.twist.twist.linear.z;
	//Predict next state
	KF.predict(u);
	odomOut.pose.pose.position.x = KF.X(0);
	odomOut.pose.pose.position.y = KF.X(1);
	odomOut.pose.pose.position.z = KF.X(2);
	odomOut.header.stamp = ros::Time::now();
	odomPublisher.publish(odomOut);

	nav_msgs::Odometry* odom = &odomOut;
	// ROS_INFO("[StateEstimator] Odom published.");
	// ROS_INFO("Seq: [%d]", odom->header.seq);
	// ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odom->pose.pose.position.x,odom->pose.pose.position.y, odom->pose.pose.position.z);
	// ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
	// ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", odom->twist.twist.linear.x,odom->twist.twist.angular.z); 
}
