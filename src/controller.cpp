/**
 * @file controller.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief Controller ROS Node
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

#include "racing_drone/controller_core.hpp"


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "controller");

  //Load parameters
  int controller_rate;
  int prediction_horizon;
  double prediction_time_step;
  std::string odom_sub_topic, ref_sub_topic, cmd_pub_topic;
  std::vector<double> P;
  double max_angle;
  double max_thrust;

  ros::param::get("controller/controller_rate", controller_rate);
  ros::param::get("controller/prediction_horizon", prediction_horizon);
  ros::param::get("controller/prediction_time_step", prediction_time_step);
  ros::param::get("controller/odom_sub_topic", odom_sub_topic);
  ros::param::get("controller/ref_sub_topic", ref_sub_topic);
  ros::param::get("controller/cmd_pub_topic", cmd_pub_topic);
  ros::param::get("controller/state_penalties", P);
  ros::param::get("controller/max_angle", max_angle);
  ros::param::get("controller/max_thrust", max_thrust);

  // ROS_INFO("[Controller] controller_rate: %d", controller_rate);
  // ROS_INFO("[Controller] prediction_horizon: %d", prediction_horizon);
  // ROS_INFO("[Controller] prediction_time_step: %f", prediction_time_step);
  // ROS_INFO("[Controller] odom_sub_topic: %s", odom_sub_topic.c_str());
  // ROS_INFO("[Controller] ref_sub_topic: %s", ref_sub_topic.c_str());
  // ROS_INFO("[Controller] cmd_pub_topic: %s", cmd_pub_topic.c_str());
  // ROS_INFO("[Controller] state_penalties: %f, %f", P[0], P[8]);


  Controller controller(controller_rate, prediction_horizon, P, prediction_time_step, max_angle, max_thrust,
                        odom_sub_topic, ref_sub_topic, cmd_pub_topic);

  ros::Rate conRate(controller_rate); // Tell ROS how fast to run this node.

  //Wait for other nodes to initialize
  // ros::Rate sleepRate(0.2);
	// sleepRate.sleep();

  // Main loop.
  while (controller.nh.ok())
  {
    ros::spinOnce();
    controller.publishControlInput();
    conRate.sleep();
  }

  return 0;
} // end main()
