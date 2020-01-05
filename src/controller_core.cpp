/**
 * @file controller_core.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief 
 * @version 0.1
 * @date 01-05-2020
 * 
 * @copyright Copyright (c) 2020
 * 
 * [License statements go here..]
 */

#include "racing_drone/controller_core.hpp"


/**
 * @brief Converts roll-pitch-yaw angles to quaternion
 * 
 * @param rpy rpy-vector [r,p,y] (in radians)
 * @param quat quaternion-vector [x,y,z,w]
 */
void rpy2quat(const std::vector<double>& rpy, std::vector<double>& quat)
{
	double cy = cos(rpy[2] * 0.5);
    double sy = sin(rpy[2] * 0.5);
    double cp = cos(rpy[1] * 0.5);
    double sp = sin(rpy[1] * 0.5);
    double cr = cos(rpy[0] * 0.5);
    double sr = sin(rpy[0] * 0.5);
    
    quat[0] = cy * cp * sr - sy * sp * cr;
    quat[1] = sy * cp * sr + cy * sp * cr;
    quat[2] = sy * cp * cr - cy * sp * sr;
	quat[3] = cy * cp * cr + sy * sp * sr;

	return;
}

// Convert quaternion to euler angles (angles in radians)
// def quat2rpy(q):
// 	q = [x,y,z,w]
// 	euler = [roll,pitch,yaw]
void quat2rpy(const std::vector<double>& q, std::vector<double>& rpy )
{
	rpy[0] = atan2( 2*(q[3]*q[0] + q[1]*q[2]), 1 - 2*(q[0]*q[0] + q[1]*q[1]) );
	rpy[1] = asin( 2*(q[3]*q[1] - q[0]*q[2]) );
	rpy[2] = atan2( 2*(q[3]*q[2] + q[0]*q[1]), 1 - 2*(q[1]*q[1]+ q[2]*q[2]) );
	return;
}

//Convert refPose to refState [Need to change to refOdom for velocity]
void pose2state(const geometry_msgs::Pose& pose, std::vector<double>& refSt)
{
	// Position
	refSt[0] = pose.position.x;
	refSt[1] = pose.position.y;
	refSt[2] = pose.position.z;

	std::vector<double> quat(4), rpy(3);
	quat[0] = pose.orientation.x;
	quat[1] = pose.orientation.y;
	quat[2] = pose.orientation.z;
	quat[3] = pose.orientation.w;

	quat2rpy(quat, rpy);

	// Velocity
	refSt[3] = 0.0;
	refSt[4] = 0.0;
	refSt[5] = 0.0;

	// Orientation (euler -> radians)
	refSt[6] = rpy[0];
	refSt[7] = rpy[1];
	refSt[8] = rpy[2];
}



//------------------------- Controller -------------------------------
Controller::Controller(int rt, int n, std::vector<double> p, double dt_,
			const std::string& odomTopic, const std::string& refTopic, const std::string& cmdTopic)
			 : rate(rt), quad_mpc(n, p, dt_), odomSubTopic(odomTopic), refSubTopic(refTopic), cmdPubTopic(cmdTopic)
{
	currState.resize(9);
	refState.resize(9);

	odomSubscriber = nh.subscribe(odomSubTopic, 10, &Controller::odomCallback, this);
	refSubscriber = nh.subscribe(refSubTopic, 10, &Controller::refCallback, this);
	cmdPublisher = nh.advertise<geometry_msgs::Twist>(cmdPubTopic, 10);

	geometry_msgs::Pose startPose;
	startPose.position.z = 1.0;
	startPose.orientation.w = 1.0;
	refPose = startPose;
	pose2state(refPose, refState);

}

Controller::~Controller()
{
	
}

void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	state_mutex.lock();

	currState[0] = odom->pose.pose.position.x;
	currState[1] = odom->pose.pose.position.y;
	currState[2] = odom->pose.pose.position.z;
	currState[3] = odom->twist.twist.linear.x;
	currState[4] = odom->twist.twist.linear.y;
	currState[5] = odom->twist.twist.linear.z;

	std::vector<double> quat(4), rpy(3);
	quat[0] = odom->pose.pose.orientation.x;
	quat[1] = odom->pose.pose.orientation.y;
	quat[2] = odom->pose.pose.orientation.z;
	quat[3] = odom->pose.pose.orientation.w;
	quat2rpy(quat, rpy);
	currState[6] = rpy[0];
	currState[7] = rpy[1];
	currState[8] = rpy[2];

	state_mutex.unlock();
}

void Controller::refCallback(const geometry_msgs::Pose::ConstPtr& refPse)
{
	refPose = *refPse;
	pose2state(refPose, refState);

	// ROS_INFO("\nReference x: %f, y: %f, z: %f\n vx: %f, vy: %f, vz: %f\n R: %f, P: %f, Y: %f\n", refState[0], refState[1], refState[2],
	//  refState[3], refState[4], refState[5], refState[6], refState[7], refState[8]);

}

void Controller::computeControlInput(void)
{
	// ROS_INFO("\nReference x: %f, y: %f, z: %f\n vx: %f, vy: %f, vz: %f\n R: %f, P: %f, Y: %f\n", refState[0], refState[1], refState[2],
	// 			refState[3], refState[4], refState[5], refState[6], refState[7], refState[8]);

	state_mutex.lock();
	std::vector<double> x0(9), xRef(9);
	std::fill(xRef.begin(), xRef.end(), 0.0);
	std::fill(x0.begin(), x0.end(), 0.0);
	std::copy(currState.begin(), currState.end() - 3, x0.begin());
	std::copy(refState.begin(), refState.end() - 3, xRef.begin()); //refState[0->5] -> xN[0->5]

	// ROS_INFO("\nReference x: %f, y: %f, z: %f\n vx: %f, vy: %f, vz: %f\n R: %f, P: %f, Y: %f\n", xRef[0], xRef[1], xRef[2],
	// 			xRef[3], xRef[4], xRef[5], xRef[6], xRef[7], xRef[8]);

	state_mutex.unlock();
	quad_mpc.optimize(x0, xRef, currState[8]);
	

	// Velocity control
	double yaw_control = 0.5 * (refState[8] - currState[8]);
	double thrust = 0.2 * (refState[2] - currState[2]);
	// double pitch_moment = 5.0 * (quad_mpc.sol_x[6] - currState[6]);
	// double roll_moment = 5.0 * (quad_mpc.sol_x[7] - currState[7]);

	// quad_mpc.sol_x =  [x,y,z,vx,vy,vz,pitch,roll,thrust]
	controlInput.linear.x = quad_mpc.sol_x[7]; // roll
	controlInput.linear.y = quad_mpc.sol_x[6]; //pitch
	controlInput.linear.z = quad_mpc.sol_x[8]; // thrust
	// controlInput.linear.z = thrust; // thrust
	// controlInput.angular.x = pitch_moment;
	// controlInput.angular.y = roll_moment;
	controlInput.angular.z = yaw_control; // yaw

}

double Controller::computeStateError(void)
{
	double error;
	for(int i=0; i<6; i++)
		error += (refState[i] - currState[i])*(refState[i] - currState[i]);
	
	return sqrt(error);
}

double  Controller::computeDistanceError(void)
{
	double error;
	for(int i=0; i<3; i++)
		error += (refState[i] - currState[i])*(refState[i] - currState[i]);
	
	error = sqrt(error);

	ROS_INFO("Distance error: %f", error);
	return error;
}

double  Controller::computeVelocityError(void)
{
	double error;
	for(int i=3; i<6; i++)
		error += (refState[i] - currState[i])*(refState[i] - currState[i]);
	
	return sqrt(error);
}

void Controller::publishControlInput(void)
{	
	ros::Time begin = ros::Time::now();
    computeControlInput();
    ros::Time end = ros::Time::now();
	double loopTime = end.toNSec() - begin.toNSec();
	ROS_INFO( "\nControl loop time: %f ms\n", loopTime/1e6);

	cmdPublisher.publish(controlInput);
		
	// ROS_INFO( "\nControl Published: theta: %f, phi: %f, T: %f, yaw: %f \n", controlInput.linear.y,
	// 		 controlInput.linear.x, controlInput.linear.z, controlInput.angular.z);
}