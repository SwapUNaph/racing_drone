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
	std::string pubTopic;
	std::string subTopic;
	int rate;
	KalmanFilter KF;
	nav_msgs::Odometry odomOut;
	
	StateEstimator(const ros::NodeHandle &node_handle,
					const std::string& pub_topic,
					const std::string& sub_topic,
					int rt, KalmanFilter kf);
	~StateEstimator();
	
	void init(void);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
	void estimatorLoopTimerCallback(const ros::TimerEvent& timerEvent);
	void publishOdometry(void);
};


