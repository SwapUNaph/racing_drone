#include "racing_drone/state_estimator_core.hpp"



int main(int argc, char **argv)
{  
  // Set up ROS.
  ros::init(argc, argv, "state_estimator");
  
  //Load parameters
  int estimator_rate;
  std::string odom_sub_topic, odom_pub_topic;
  std::vector<double> Q_diag, R_diag;

  ros::param::get("state_estimator/estimator_rate", estimator_rate);
  ros::param::get("state_estimator/odom_sub_topic", odom_sub_topic);
  ros::param::get("state_estimator/odom_pub_topic", odom_pub_topic);
  ros::param::get("state_estimator/cov_meas_noise", R_diag);
  ros::param::get("state_estimator/cov_process_noise", Q_diag);

  ROS_INFO("[State-estimator] estimator_rate: %d", estimator_rate);
  ROS_INFO("[State-estimator] odom_sub_topic: %s", odom_sub_topic.c_str());
  ROS_INFO("[State-estimator] odom_pub_topic: %s", odom_pub_topic.c_str());
  ROS_INFO("[State-estimator] cov_meas_noise: %f", R_diag[0]);
  ROS_INFO("[State-estimator] cov_process_noise: %f", Q_diag[0]);

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
	double dt = 1.0/estimator_rate;
	
	for(int i=0; i<3; i++){
		x0(i) = 0.0;
		for(int j=0; j<3; j++)
		{
			f(i,j) = 0.0;
			b(i,j) = 0.0;
			h(i,j) = 0.0;
			q(i,j) = 0.0;
			r(i,j) = 0.0;
		}
	}

	for(int i=0; i<n; i++)
	{
		f(i,i) = 1.0;
		h(i,i) = 1.0;
		q(i,i) = Q_diag[i];
		// ROS_INFO(" q(%d,%d): %f",i,i,q(i,i));
	} 
	
	for(int i=0; i<p; i++)
	{
		r(i,i) = R_diag[i];
		// ROS_INFO(" r(%d,%d): %f",i,i,r(i,i));
	} 
	
	b(0,0) = dt;
	b(1,1) = dt;
	b(2,2) = dt;
	
	KalmanFilter KF(f,b,h,q,r,x0);

	//Node handles
	ros::NodeHandle nh("");

	//State-Estimator
	StateEstimator estimator(nh, odom_pub_topic, odom_sub_topic, estimator_rate, KF); 

	//Wait for other nodes to initialize
	ros::Rate sleepRate(0.5);
	sleepRate.sleep();
	
	//Multithreaded ROS spinner
	ros::MultiThreadedSpinner multiSpinner(4);
	ROS_INFO("State-estimator initiated...");
	ros::spin(multiSpinner);

	return 0;
} // end main()
