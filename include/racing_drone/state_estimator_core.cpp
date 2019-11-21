#include "state_estimator_core.hpp"

StateEstimator::StateEstimator()
{
	string pubTopic = "/state_estimator/odom";
	string subTopic = "/ground_truth/state";
	odomSubscriber = nh.subscribe(subTopic, 10, &StateEstimator::odomCallback, this);
	odomPublisher = nh.advertise<nav_msgs::Odometry>(pubTopic, 10);
}

StateEstimator::~StateEstimator()
{

}

void StateEstimator::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	odomIn = *odom;
	//ROS_INFO("Seq: [%d]", odom->header.seq);
	//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odom->pose.pose.position.x,odom->pose.pose.position.y, odom->pose.pose.position.z);
	//ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
	//ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", odom->twist.twist.linear.x,odom->twist.twist.angular.z);
}

void StateEstimator::publishOdometry(void)
{
	odomPublisher.publish(odomIn);
}
