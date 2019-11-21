#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>

using std::string;

class StateEstimator
{
public:
	ros::Publisher odomPublisher;
	ros::Subscriber odomSubscriber;
	ros::NodeHandle nh;
	nav_msgs::Odometry odomIn;
	
	StateEstimator();
	~StateEstimator();
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
	void publishOdometry(void);
};
