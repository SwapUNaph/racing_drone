#include "racing_drone/state_estimator_core.hpp"
#include "nav_msgs/Odometry.h"


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


	//Node handles
	ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

	//State-Estimator
	StateEstimator estimator(nh, nh_private, 50, KF); 

	ros::MultiThreadedSpinner multiSpinner(4);
	ROS_INFO("State-estimator initiated...");
	ros::spin(multiSpinner);
	return 0;
} // end main()
