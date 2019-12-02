#include "racing_drone/controller_core.hpp"


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "controller");

  int control_loop_rate = 10;
  int prediction_horizon = 5;
  double prediction_time_step = 0.5;
  std::vector<double> P{ 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 0.5, 0.5, 0.5 };
  
  Controller controller(control_loop_rate, prediction_horizon, P, prediction_time_step);

  ros::Rate r(controller.rate); // Tell ROS how fast to run this node.

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
    r.sleep();
  }

  return 0;
} // end main()
