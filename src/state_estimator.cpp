#include "racing_drone/state_estimator_core.hpp"
#include "nav_msgs/Odometry.h"
/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

void odomClback(const nav_msgs::Odometry::ConstPtr& odom)
{
	ROS_INFO("Seq: [%d]", odom->header.seq);
	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odom->pose.pose.position.x,odom->pose.pose.position.y, odom->pose.pose.position.z);
	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", odom->twist.twist.linear.x,odom->twist.twist.angular.z);
}


int main(int argc, char **argv)
{  
  // Set up ROS.
  ros::init(argc, argv, "state_estimator");
  
  //Kalman Filter 
  	int n=3; //States
	int m=3; //Inputs
	int p=3; //Outputs
	
	matrix<double> f(3,3);
	matrix<double> b(3,3);
	matrix<double> h(3,3);
	matrix<double> q(3,3);
	matrix<double> r(3,3);
	vector<double> x0(3);
	double dt = 1/500.0;
	
	for(int i=0; i<n; i++)
	{
		f(i,i) = 1.0;
		h(i,i) = 1.0;
		q(i,i) = 1e-2;
	} 
	
	for(int i=0; i<p; i++)
	{
		r(i,i) = 0.1;
	} 
	
	b(0,0) = dt;
	b(1,1) = dt;
	b(2,2) = dt;
	
	KalmanFilter KF(f,b,h,q,r,x0);
	
	 //State-Estimator
	 StateEstimator estimator(500.0, KF); //200 Hz 

	  // Tell ROS how fast to run this node.
	  ros::Rate rosRate(estimator.rate); 

	  // Main loop.
	  while (estimator.nh.ok())
	  {
		ros::spinOnce();
		estimator.publishOdometry();
		rosRate.sleep();
	  }

	  return 0;
} // end main()
