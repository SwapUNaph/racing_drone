#include "racing_drone/state_estimator_core.hpp"


//--------------------- State Estimator ------------------------------------------
StateEstimator::StateEstimator(int rt, KalmanFilter kf) : rate(rt), KF(kf)
{
	std::string pubTopic = "/state_estimator/odom";
	std::string subTopic = "/ground_truth/state";
	odomSubscriber = nh.subscribe(subTopic, 10, &StateEstimator::odomCallback, this);
	odomPublisher = nh.advertise<nav_msgs::Odometry>(pubTopic, 10);
}

StateEstimator::~StateEstimator()
{

}

void StateEstimator::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	odomOut = *odom;
	vector<double> y(3);
	y(0) = odom->pose.pose.position.x;
	y(1) = odom->pose.pose.position.y;
	y(2) = odom->pose.pose.position.z;
	
	
	//Examples
	//ROS_INFO("Seq: [%d]", odom->header.seq);
	//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odom->pose.pose.position.x,odom->pose.pose.position.y, odom->pose.pose.position.z);
	//ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
	//ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", odom->twist.twist.linear.x,odom->twist.twist.angular.z); 
	
	//Update the estimate
	KF.update(y);
	odomOut.pose.pose.position.x = KF.X(0);
	odomOut.pose.pose.position.y = KF.X(1);
	odomOut.pose.pose.position.z = KF.X(2);

}

void StateEstimator::publishOdometry(void)
{
	vector<double> u(3);
	u(0) = odomOut.twist.twist.linear.x;
	u(1) = odomOut.twist.twist.linear.y;
	u(2) = odomOut.twist.twist.linear.z;
	KF.predict(u);
	odomOut.pose.pose.position.x = KF.X(0);
	odomOut.pose.pose.position.y = KF.X(1);
	odomOut.pose.pose.position.z = KF.X(2);
	
	odomPublisher.publish(odomOut);
}
