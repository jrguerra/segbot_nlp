#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <iostream>

// --screen

void amclHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float z = msg->pose.pose.position.z;

	float i = msg->pose.pose.orientation.x;
	float j = msg->pose.pose.orientation.y;
	float k = msg->pose.pose.orientation.z;
	float w = msg->pose.pose.orientation.w;

	ROS_INFO("X:%f Y:%f Z:%f I:%f J:%f K:%f W:%f", x, y, z, i, j, k, w);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "command_module");
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ros::Subscriber sub = n.subscribe("amcl_pose", 1000, amclHandler);

	ros::Rate loop_rate(10);

	int count = 0;
//	int x;
	geometry_msgs::PoseStamped p;

	geometry_msgs::Twist x;

	x.linear.x = 20;
	x.angular.z = 0;

	chatter_pub.publish(x);
	while (ros::ok()) {
		//std:cin >> x;
	//	ROS_INFO("I am alive. Sending out move_base_simple/goal");

/*
		p.header.seq = count;
		p.header.frame_id = "1";

		p.pose.position.x = 10;
		p.pose.position.y = 10;
		p.pose.position.z = 0;

		//p.pose.orientation.x = 0;
		//p.pose.orientation.y = 0;
		p.pose.orientation.z = 0.5;

		p.pose.orientation.w = 0.5;
*/
		geometry_msgs::Twist x;
		x.linear.x = 0.1;
		x.angular.z = 0;

		chatter_pub.publish(x);
 

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
