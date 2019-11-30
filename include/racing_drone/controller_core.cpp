#include "controller_core.hpp"


//------------------------- Controller -------------------------------
Controller::Controller(int rt, int n, std::vector<double> p) : rate(rt), quad_mpc(n, p, 1.0/rt)
{
	currState.resize(9);
	std::string pubTopic = "/cmd_vel";
	std::string odom_subTopic = "/ground_truth/state";
	std::string ref_subTopic = "/controller/reference";
	odomSubscriber = nh.subscribe(odom_subTopic, 10, &Controller::odomCallback, this);
	refSubscriber = nh.subscribe(ref_subTopic, 10, &Controller::refCallback, this);
	cmdPublisher = nh.advertise<geometry_msgs::Twist>(pubTopic, 10);
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
	R.getRPY(&currState[6], &currState[7], &currState[8]);
}

void Controller::refCallback(const geometry_msgs::Pose::ConstPtr& refPse)
{
	refPose = *refPse;
}

void Controller::computeControlInput(void)
{
	vector<double> x0(9), xN(9);
	std::copy(currState.begin(), currStat.end()-3, x0.begin());
	xN[0] = refPose.position.x;
	xN[1] = refPose.position.y;
	xN[2] = refPose.position.z;
	quad_mpc.optimize(x0, xN, currState[8]);

	tf::Quaternion q(refPose.orientation.x, refPose.orientation.y,
					refPose.orientation.z, refPose.orientation.w);
	tf::Matrix3x3 R(q);
	vector<double> rpy(3);
	R.getRPY(&rpy[0], &rpy[1], &rpy[2]);

	double yaw_control = 0.5 * (rpy[2] - currState[8]);

	controlInput.linear.x = quad_mpc.sol_x[0];
	controlInput.linear.y = - quad_mpc.sol_x[1];
	controlInput.linear.z = quad_mpc.sol_x[2];
	controlInput.angular.z = yaw_control;
}

void Controller::publishControlInput(void)
{
	cmdPublisher.publish(controlInput);
}