/**
 * @file state_estimator_core.hpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief state_estimator_core declaration 
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


