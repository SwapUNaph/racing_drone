#include "racing_drone/controller_core.hpp"


//------------------------- Controller -------------------------------
Controller::Controller(int rt, int n, std::vector<double> p, double dt_,
			const std::string& odomTopic, const std::string& refTopic, const std::string& cmdTopic)
			 : rate(rt), quad_mpc(n, p, dt_), odomSubTopic(odomTopic), refSubTopic(refTopic), cmdPubTopic(cmdTopic)
{
	currState.resize(9);
	odomSubscriber = nh.subscribe(odomSubTopic, 10, &Controller::odomCallback, this);
	refSubscriber = nh.subscribe(refSubTopic, 10, &Controller::refCallback, this);
	cmdPublisher = nh.advertise<geometry_msgs::Twist>(cmdPubTopic, 10);
	geometry_msgs::Pose startPose;
	startPose.position.z = 1.0;
	startPose.orientation.w = 1.0;
	refPose = startPose;
}

Controller::~Controller()
{
	
}

void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	currState[0] = odom->pose.pose.position.x;
	currState[1] = odom->pose.pose.position.y;
	currState[2] = odom->pose.pose.position.z;
	currState[3] = odom->twist.twist.linear.x;
	currState[4] = odom->twist.twist.linear.y;
	currState[5] = odom->twist.twist.linear.z;
	tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
					odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
	tf::Matrix3x3 R(q);
	R.getRPY(currState[6], currState[7], currState[8]);
}

void Controller::refCallback(const geometry_msgs::Pose::ConstPtr& refPse)
{
	refPose = *refPse;
	//ROS_INFO("Reference Pose: ")
}

void Controller::computeControlInput(void)
{
	std::vector<double> x0(9), xN(9);
	std::copy(currState.begin(), currState.end()-3, x0.begin());
	xN[0] = refPose.position.x;
	xN[1] = refPose.position.y;
	xN[2] = refPose.position.z;
	quad_mpc.optimize(x0, xN, currState[8]);

	tf::Quaternion q(refPose.orientation.x, refPose.orientation.y,
					refPose.orientation.z, refPose.orientation.w);
	tf::Matrix3x3 R(q);
	std::vector<double> rpy(3);
	R.getRPY(rpy[0], rpy[1], rpy[2]);
	// ROS_INFO("\nReference x: %f, y: %f, z: %f, R: %f, P: %f, Y: %f", xN[0], xN[1], xN[2], rpy[0], rpy[1], rpy[2]);

	// Velocity control
	double yaw_control = 0.5 * (rpy[2] - currState[8]);
	double thrust = 0.5 * (xN[2] - currState[2]);
	// double pitch_moment = 5.0 * (quad_mpc.sol_x[6] - currState[6]);
	// double roll_moment = 5.0 * (quad_mpc.sol_x[7] - currState[7]);

	controlInput.linear.x = 5.0 * quad_mpc.sol_x[6];
	controlInput.linear.y = - 5.0 * quad_mpc.sol_x[7];
	controlInput.linear.z = thrust;
	// controlInput.angular.x = pitch_moment;
	// controlInput.angular.y = roll_moment;
	controlInput.angular.z = yaw_control;
}

void Controller::publishControlInput(void)
{	
	cmdPublisher.publish(controlInput);
	// ROS_INFO( "\nControl Published: theta: %f, phi: %f, T: %f, yaw: %f \n", controlInput.linear.x,
	// 		 controlInput.linear.y, controlInput.linear.z, controlInput.angular.z);
}