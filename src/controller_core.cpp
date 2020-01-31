/**
 * @file controller_core.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief controller_core definition
 * @version 0.1
 * @date 01-06-2020
 * 
 *  Copyright (c) 2020 Swapneel Naphade
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#include "racing_drone/controller_core.hpp"
#include "common.cpp"

/**
 * @brief Construct a new Controller:: Controller object
 * 
 * @param rt Controller rate (in Hz)
 * @param n Prediction Horizon for MPC
 * @param p State Penalties for MPC [Qx,Qy,Qz,Qvx,Qvy,Qvz,Rpitch,Rroll,Rthrust]
 * @param dt_ Time step for MPC (in seconds)
 * @param odomTopic Odometry In topic (Odometry)
 * @param refTopic Reference State topic (DroneState)
 * @param cmdTopic Control Command Topic (Twist)
 */

Controller::Controller(int rt, int n, std::vector<double> p, double dt_, double maxAng_, double maxThrust_,
			const std::string& odomTopic, const std::string& refTopic, const std::string& cmdTopic, bool bebop_ )
			 : rate(rt), quad_mpc(n, p, dt_, maxAng_, maxThrust_), odomSubTopic(odomTopic), refSubTopic(refTopic), cmdPubTopic(cmdTopic), bebop(bebop_)
{
	currState.resize(9);
	refState.resize(9);

	odomSubscriber = nh.subscribe(odomSubTopic, 10, &Controller::odomCallback, this);
	refSubscriber = nh.subscribe(refSubTopic, 10, &Controller::refCallback, this);
	enCtrlSubscriber = nh.subscribe("/auto/autonomy_active", 10, &Controller::enCtrlCallback, this);
	cmdPublisher = nh.advertise<geometry_msgs::Twist>(cmdPubTopic, 10);
	controlTimer = nh.createTimer(ros::Duration(1.0/rate), &Controller::controllerTimerCallback, this);

	std::fill(refState.begin(), refState.end(), 0.0);
	refState[2] = 1.0;

	enable_control = true;
}

/**
 * @brief Destroy the Controller:: Controller object
 * 
 */
Controller::~Controller()
{
	
}

/**
 * @brief Callback for Odometry In, updates current state
 * 
 * @param odom 
 */
void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	state_mutex.lock();

	currState[0] = odom->pose.pose.position.x;
	currState[1] = odom->pose.pose.position.y;
	currState[2] = odom->pose.pose.position.z;

	std::vector<double> quat(4), rpy(3);
	quat[0] = odom->pose.pose.orientation.x;
	quat[1] = odom->pose.pose.orientation.y;
	quat[2] = odom->pose.pose.orientation.z;
	quat[3] = odom->pose.pose.orientation.w;
	quat2rpy(quat, rpy);
	currState[6] = rpy[0];
	currState[7] = rpy[1];
	currState[8] = rpy[2];

	if( bebop )
	{
		// bebop publishes body velocity
		std::vector<double> body_vel(3);
		body_vel[0] = odom->twist.twist.linear.x;
		body_vel[1] = odom->twist.twist.linear.y;
		body_vel[2] = odom->twist.twist.linear.z;

		body_vel = rotateVec(quat, body_vel); //Transforming to inertial frame

		currState[3] = body_vel[0];
		currState[4] = body_vel[1];
		currState[5] = body_vel[2];

	}
	else
	{
		currState[3] = odom->twist.twist.linear.x;
		currState[4] = odom->twist.twist.linear.y;
		currState[5] = odom->twist.twist.linear.z;
	}
	

	state_mutex.unlock();
	// ROS_INFO( "Odom Callback Success." );
}

/**
 * @brief Callback for reference state, updates reference state
 * 
 * @param refDroneState 
 */
void Controller::refCallback(const racing_drone::DroneState::ConstPtr& refDroneState)
{
	refState[0] = refDroneState->position.x; //x
	refState[1] = refDroneState->position.y; //y
	refState[2] = refDroneState->position.z; //z
	refState[3] = refDroneState->velocity.x; //vx
	refState[4] = refDroneState->velocity.y; //vy
	refState[5] = refDroneState->velocity.z; //vz
	refState[6] = 0.0;						 //roll
	refState[7] = 0.0;						 //pitch
	refState[8] = refDroneState->yaw;		 //yaw

	// ROS_INFO("\nReference x: %f, y: %f, z: %f\n vx: %f, vy: %f, vz: %f\n R: %f, P: %f, Y: %f\n", refState[0], refState[1], refState[2],
	//  refState[3], refState[4], refState[5], refState[6], refState[7], refState[8]);

}

/**
 * @brief Set the enable control variable
 * 
 * @param enCtrl 
 */
void Controller::enCtrlCallback(const std_msgs::Bool::ConstPtr& enCtrl)
{
	enable_control = enCtrl->data;
}

/**
 * @brief Computes Control Input using MPC
 * 
 */
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
	
	xRef[8] = 9.8; // Thrust reference

	// ROS_INFO("\nReference x: %f, y: %f, z: %f\n vx: %f, vy: %f, vz: %f\n R: %f, P: %f, Y: %f\n", xRef[0], xRef[1], xRef[2],
	// 			xRef[3], xRef[4], xRef[5], xRef[6], xRef[7], xRef[8]);

	state_mutex.unlock();
	quad_mpc.optimize(x0, xRef, currState[8]);
	

	// Yaw control
	std::vector<double> refQuat(4), currQuat(4), diffQuat(4),
						refRPY(3), currRPY(3), diffRPY(3);

	std::copy(refState.begin()+6, refState.end(), refRPY.begin());
	std::copy(currState.begin()+6, currState.end(), currRPY.begin());

	rpy2quat(refRPY, refQuat);
	rpy2quat(currRPY, currQuat);

	diffQuat = quatDifference(refQuat, currQuat);
	quat2rpy(diffQuat, diffRPY);

	double yaw_control = 1.0 * diffRPY[2];
	double thrust = 1.0 * (refState[2] - currState[2]);
	// double pitch_moment = 5.0 * (quad_mpc.sol_x[6] - currState[6]);
	// double roll_moment = 5.0 * (quad_mpc.sol_x[7] - currState[7]);

	// quad_mpc.sol_x =  [x,y,z,vx,vy,vz,pitch,roll,thrust]

	if( bebop )
	{
		// Bebop2 quad control input
		controlInput.linear.x = quad_mpc.sol_x[6] * 180.0 / PI / 20.0; // pitch
		controlInput.linear.y = - quad_mpc.sol_x[7] * 180.0 / PI / 20.0; // roll
		// controlInput.linear.z = 0.5 * (quad_mpc.sol_x[8] - 9.8); // thrust
		controlInput.linear.z = thrust; // thrust
		// controlInput.angular.x = pitch_moment;
		// controlInput.angular.y = roll_moment;
		controlInput.angular.z = yaw_control; // yaw
	}
	else
	{
		// Simulation quad control input
		controlInput.linear.x = quad_mpc.sol_x[7]; // roll
		controlInput.linear.y = quad_mpc.sol_x[6]; //pitch
		controlInput.linear.z = quad_mpc.sol_x[8]; // thrust
		// controlInput.linear.z = thrust; // thrust
		// controlInput.angular.x = pitch_moment;
		// controlInput.angular.y = roll_moment;
		controlInput.angular.z = yaw_control; // yaw
	}
	



}

void Controller::controllerTimerCallback(const ros::TimerEvent& timerEvent)
{
	publishControlInput();
}

/**
 * @brief Computes Euclidean distance between refState and currState
 * 
 * @return double state error
 */
double Controller::computeStateError(void)
{
	double error;
	for(int i=0; i<6; i++)
		error += (refState[i] - currState[i])*(refState[i] - currState[i]);
	
	return sqrt(error);
}

/**
 * @brief Computes Euclidean distance between refState position and currState position
 * 
 * @return double Distance error
 */
double  Controller::computeDistanceError(void)
{
	double error;
	for(int i=0; i<3; i++)
		error += (refState[i] - currState[i])*(refState[i] - currState[i]);
	
	error = sqrt(error);

	ROS_INFO("Distance error: %f", error);
	return error;
}

/**
 * @brief Computes Euclidean distance between refState velocity and currState velocity
 * 
 * @return double Velocity error
 */
double  Controller::computeVelocityError(void)
{
	double error;
	for(int i=3; i<6; i++)
		error += (refState[i] - currState[i])*(refState[i] - currState[i]);
	
	return sqrt(error);
}

/**
 * @brief Computes and publishes control input
 * 
 */
void Controller::publishControlInput(void)
{	
	if( enable_control )
	{
		ros::Time begin = ros::Time::now();
		computeControlInput();
		ros::Time end = ros::Time::now();
		double loopTime = end.toNSec() - begin.toNSec();
		ROS_INFO( "Control compute time: %.0f ms\n", loopTime/1e6);
		cmdPublisher.publish(controlInput);			
	}
	else
	{
		// geometry_msgs::Twist zero_state;
		// cmdPublisher.publish(zero_state);
	}
	

		
	// ROS_INFO( "\nControl Published: theta: %f, phi: %f, T: %f, yaw: %f \n", controlInput.linear.y,
	// 		 controlInput.linear.x, controlInput.linear.z, controlInput.angular.z);
}
