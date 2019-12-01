#include "racing_drone/controller_core.hpp"


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "controller");

  int control_loop_rate = 10;
  int prediction_horizon = 10;
  double prediction_time_step = 0.2;
  std::vector<double> P{ 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 0.5, 0.5, 0.5 };
  
  Controller controller(control_loop_rate, prediction_horizon, P, prediction_time_step);

  ros::Rate r(controller.rate); // Tell ROS how fast to run this node.

  // Main loop.
  while (controller.nh.ok())
  {
    ros::spinOnce();
    controller.computeControlInput();
    controller.publishControlInput();
    r.sleep();
  }

  return 0;
} // end main()
