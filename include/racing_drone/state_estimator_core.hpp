#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include "KalmanFilter.hpp"

class StateEstimator
{
public:
	ros::Publisher odomPublisher;
	ros::Subscriber odomSubscriber;
	ros::NodeHandle nh;
	int rate;
	KalmanFilter KF;
	nav_msgs::Odometry odomOut;
	
	StateEstimator(int rt, KalmanFilter kf);
	~StateEstimator();
	
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
	void publishOdometry(void);
};


