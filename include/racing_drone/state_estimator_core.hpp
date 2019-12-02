#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include "KalmanFilter.hpp"

class StateEstimator
{
public:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
	ros::Publisher odomPublisher;
	ros::Subscriber odomSubscriber;
	ros::Timer estimatorLoopTimer;
	int rate;
	KalmanFilter KF;
	nav_msgs::Odometry odomOut;
	
	StateEstimator(const ros::NodeHandle &node_handle,
					const ros::NodeHandle &private_node_handle, 
					int rt, KalmanFilter kf);
	~StateEstimator();
	
	void init(void);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
	void estimatorLoopTimerCallback(const ros::TimerEvent& timerEvent);
	void publishOdometry(void);
};


