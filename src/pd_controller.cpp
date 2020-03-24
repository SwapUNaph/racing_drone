#include "racing_drone/PID.hpp"
#include <ros/ros.h>
#include <ros/time.h>
#include <racing_drone/DroneState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "common.cpp"


namespace ublas = boost::numeric::ublas;
void odomCallback(nav_msgs::Odometry::ConstPtr odom);
void referenceCallback(racing_drone::DroneState::ConstPtr refDroneState);
void controlCallback(const ros::TimerEvent& event);
ros::Subscriber odomSub;
ros::Subscriber refSub;
ros::Publisher cmdPub;
ros::Timer controlTimer;

std::vector<double> currState(9);
std::vector<double> refState(9);
std::vector<double> quat(4);


void odomCallback(nav_msgs::Odometry::ConstPtr odom)
{
    currState[0] = odom->pose.pose.position.x;
	currState[1] = odom->pose.pose.position.y;
	currState[2] = odom->pose.pose.position.z;
    currState[3] = odom->twist.twist.linear.x;
    currState[4] = odom->twist.twist.linear.y;
    currState[5] = odom->twist.twist.linear.z;
	std::vector<double> rpy(3);
	quat[0] = odom->pose.pose.orientation.x;
	quat[1] = odom->pose.pose.orientation.y;
	quat[2] = odom->pose.pose.orientation.z;
	quat[3] = odom->pose.pose.orientation.w;
	quat2rpy(quat, rpy);
	currState[6] = rpy[0];
	currState[7] = rpy[1];
	currState[8] = rpy[2];
    ROS_INFO("Odom Received");
}

void referenceCallback(racing_drone::DroneState::ConstPtr refDroneState)
{
    refState[0] = refDroneState->position.x; //x
	refState[1] = refDroneState->position.y; //y
	refState[2] = refDroneState->position.z; //z
	refState[3] = refDroneState->velocity.x; //vx
	refState[4] = refDroneState->velocity.y; //vy
	refState[5] = refDroneState->velocity.z; //vz
	refState[6] = 0.0;						 //roll
	refState[7] = 0.0;						 //pitch
	refState[8] = refDroneState->yaw;		 //yaw.
    ROS_INFO("Reference Received");
}

void controlCallback(const ros::TimerEvent& event)
{
    geometry_msgs::Twist controlInput;
    std::vector<double> pos_error(3), vel_error(3);
    pos_error[0] = refState[0] - currState[0];
    pos_error[1] = refState[1] - currState[1];
    pos_error[2] = refState[2] - currState[2];
    vel_error[0] = refState[3] - currState[3];
    vel_error[1] = refState[4] - currState[4];
    vel_error[2] = refState[5] - currState[5];

    pos_error = rotateVec(quatConjugate(quat), pos_error);
    vel_error = rotateVec(quatConjugate(quat), vel_error);

    controlInput.linear.x = - 0.2 * pos_error[1] - 0.5 * vel_error[1]; // roll
    controlInput.linear.y = 0.2 * pos_error[0] + 0.5 * vel_error[0]; //pitch
    controlInput.linear.z = 9.8 + 1.0 * pos_error[2] + 0.5 * vel_error[2]; // specific thrust
    // controlInput.linear.z = thrust; // thrust
    // controlInput.angular.x = pitch_moment;
    // controlInput.angular.y = roll_moment;
    controlInput.angular.z = 0.5 * std::asin(std::sin(refState[8] - currState[8])); // yaw
    cmdPub.publish(controlInput);
    ROS_INFO("Control Published");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pd_controller");
    ros::NodeHandle nh("");
    odomSub = ros::Subscriber ( nh.subscribe("/ground_truth/state", 10, odomCallback) );
    refSub = ros::Subscriber ( nh.subscribe("/controller/reference", 10, referenceCallback) );
    cmdPub = ros::Publisher ( nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10) );
    controlTimer = ros::Timer ( nh.createTimer(ros::Duration(1.0/60.0), controlCallback) );

    ros::MultiThreadedSpinner multiSpinner(4);
    ros::spin(multiSpinner);
    return 0;
}

