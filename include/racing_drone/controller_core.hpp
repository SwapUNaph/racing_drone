#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

class Controller
{
public:
	ros::Publisher controlPublisher;
	ros::Subscriber refSubscriber;
	ros::Subscriber odomSubscriber;
	ros::NodeHandle nh;
	int rate; //Control loop rate, Hz 
	int N; //horizon_window
	vector<double> P; // Ccst penalties, 9*1 vector
	MPC quad_mpc;

	geometry_msgs::Twist controlInput;
	std::vector<double> currState; //x,y,z,vx,vy,vz,roll,pitch,yaw
	geometry_msgs::Pose refPose;
	
	Controller(int rt, int n, std::vector<double> p);
	~Controller();
	
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
	void refCallback(const geometry_msgs::Pose::ConstPtr& refPose);
	void computeControlInput(void);
	void publishControlInput(void);
};
