/**
 * @file controller_core.hpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief controller_core declaration
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

#pragma once

#include <algorithm>
#include <vector>
#include <mutex>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "MPC.hpp"

class Controller
{
public:
	ros::Publisher cmdPublisher; //Control Input publisher
	ros::Subscriber refSubscriber; // Reference Input subscriber
	ros::Subscriber odomSubscriber; // Odometery subscriber
	ros::NodeHandle nh; // Node handle
	std::string odomSubTopic;
	std::string refSubTopic;
	std::string cmdPubTopic;
	int rate; //Control loop rate, Hz 
	int N; //horizon_window
	std::vector<double> P; // Cost penalties, 9*1 vector
	MPC quad_mpc;	// Model Predictive Controller Object

	geometry_msgs::Twist controlInput; // Control Input message (Twist on /cmd_vel)
	std::vector<double> currState; // x,y,z,vx,vy,vz,roll,pitch,yaw
	geometry_msgs::Pose refPose; // Reference pose
	std::vector<double> refState; // x,y,z,vx,vy,vz,roll,pitch,yaw
	std::mutex state_mutex; // Mutex for locking currState access
	
	Controller(int rt, int n, std::vector<double> p, double dt_,
			const std::string& odomTopic, const std::string& refTopic, const std::string& cmdTopic); //Controller Constructor
	~Controller(); //Controller Destructor
	
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom); //Odometry Callbcak for updating current state
	void refCallback(const geometry_msgs::Pose::ConstPtr& refPose); // Reference Callback for updating reference Pose
	void computeControlInput(void); // Compute Control Input by MPC optimization
	void publishControlInput(void); // Publish Control Input 

	double computeStateError(void);
	double computeDistanceError(void);
	double computeVelocityError(void);
};
