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

  ros::param::get("controller/controller_rate", controller_rate);
  ros::param::get("controller/prediction_horizon", prediction_horizon);
  ros::param::get("controller/prediction_time_step", prediction_time_step);
  ros::param::get("controller/odom_sub_topic", odom_sub_topic);
  ros::param::get("controller/ref_sub_topic", ref_sub_topic);
  ros::param::get("controller/cmd_pub_topic", cmd_pub_topic);
  ros::param::get("controller/state_penalties", P);

  ROS_INFO("[Controller] controller_rate: %d", controller_rate);
  ROS_INFO("[Controller] prediction_horizon: %d", prediction_horizon);
  ROS_INFO("[Controller] prediction_time_step: %f", prediction_time_step);
  ROS_INFO("[Controller] odom_sub_topic: %s", odom_sub_topic.c_str());
  ROS_INFO("[Controller] ref_sub_topic: %s", ref_sub_topic.c_str());
  ROS_INFO("[Controller] cmd_pub_topic: %s", cmd_pub_topic.c_str());
  ROS_INFO("[Controller] state_penalties: %f, %f", P[0], P[8]);


  Controller controller(controller_rate, prediction_horizon, P, prediction_time_step,
                        odom_sub_topic, ref_sub_topic, cmd_pub_topic);

  ros::Rate conRate(controller_rate); // Tell ROS how fast to run this node.

  //Wait for other nodes to initialize
  ros::Rate sleepRate(0.2);
	sleepRate.sleep();

  // Main loop.
  while (controller.nh.ok())
  {
    ros::spinOnce();
    ros::Time begin = ros::Time::now();
    controller.computeControlInput();
    ros::Time end = ros::Time::now();
	  double loopTime = end.toNSec() - begin.toNSec();
    controller.publishControlInput();
    ROS_INFO( "\nControl loop time: %f ms\n", loopTime/1e6);
    conRate.sleep();
  }

  return 0;
} // end main()
