#include <ros/ros.h>
#include <ros/time.h>
#include "nav_msgs/Odometry.h"
/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

void odmClback(const nav_msgs::Odometry::ConstPtr& odom)
{
	ROS_INFO("[controller] Seq: [%d]", odom->header.seq);
	ROS_INFO("[controller]  Position-> x: [%f], y: [%f], z: [%f]", odom->pose.pose.position.x,odom->pose.pose.position.y, odom->pose.pose.position.z);
	ROS_INFO("[controller]  Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
	ROS_INFO("[controller]  Vel-> Linear: [%f], Angular: [%f]", odom->twist.twist.linear.x,odom->twist.twist.angular.z);
}


int main(int argc, char **argv)
{
  //State-Estimator
  //StateEstimator *estimator = new StateEstimator();
  
  // Set up ROS.
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  
  std::string subTopic = "/ground_truth/state";

  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  ros::Subscriber sub_message = n.subscribe(subTopic.c_str(), 1000, odmClback);

  // Tell ROS how fast to run this node.
  ros::Rate r(5); //100 Hz

  // Main loop.
  while (n.ok())
  {
	//Propagate the model
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
