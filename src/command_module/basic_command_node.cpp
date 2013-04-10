#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "segbot_nlp/VoiceCommand.h"
#include "segbot_nlp/VoiceCommandTypes.h"
#include "Interpreter.h"
#include <queue> 
#include <cmath>
#include <vector>

#define SPEED 0.25

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef ros::NodeHandle NodeHandle;
typedef ros::Publisher Publisher;
typedef ros::Subscriber Subscriber; 
typedef move_base_msgs::MoveBaseGoal MoveBaseGoal;

#define PI 3.14159

float currentX = 0.00;
float currentY = 0.00;
float currentTheta = 0.00;

struct Goal {
	float x;
  	float y;
	float z;
	float w;
	float theta;
	char dir;

	float dist;
	float dTheta;
};

std::queue<Goal> GoalFifo;
void messageHandle(const segbot_nlp::VoiceCommand::ConstPtr& msg) {
	int opcode = msg->commandCode;
	float distance = msg->distance;
	float angle = msg->angle;
	int location = msg->loc;
	int numTimes = msg->numTimes;

	float rad = (angle * PI) / 180.00;
	Goal newGoal;
	geometry_msgs::Twist newMsg;
        for (int j = 0; j < numTimes; j ++) {
  	switch (opcode) {
	  	case RC_move:
			if (angle < 0) { 
				newGoal.dir = 0;
			} else {
				newGoal.dir = 1;
			}

			newGoal.dist = distance / SPEED;
			newGoal.dTheta = (rad) / SPEED;
			if (newGoal.dTheta < 0.0) { newGoal.dTheta = - newGoal.dTheta; }

			GoalFifo.push(newGoal);
                        break;
		case RC_goBack:
                        break;

                case RC_dance:
                        break;

		default:
			break;
	}
     }

}


void odomReceived(const nav_msgs::Odometry::ConstPtr& msg) {
	float x = msg->pose.pose.position.x;
        float y = msg->pose.pose.position.y;

	float z = msg->pose.pose.orientation.z;
	float w = msg->pose.pose.orientation.w;

	if (w >= 0.00) {
		currentTheta = 2.00 * asin(z);
	} else {
		currentTheta = PI + (2.00 * asin(z));
	}
	
	float dX = x - currentX;
        float dY = y - currentY;

        if (dX > 0.01 || dX < -0.01) {
          ROS_INFO("** COMMAND MODULE: NEW X: %f", x);
        }

        if (dY > 0.01 || dY < -0.01) {
          ROS_INFO("** COMMAND MODULE: NEW Y: %f", y);
        }

        //ROS_INFO("** Command Module Position X:%f Y:%f", x, y);
	currentX = x;
        currentY = y;
}


geometry_msgs::Twist stop(void) {
	geometry_msgs::Twist retVal;
	retVal.linear.x = 0;
	retVal.linear.y = 0;
	retVal.linear.z = 0;
	
	retVal.angular.x = 0;
	retVal.angular.y = 0;
	retVal.angular.z = 0;

	return retVal;
}

geometry_msgs::Twist move(char type) {
	geometry_msgs::Twist retVal = stop();

	if (type == 0) {
        	retVal.linear.x = SPEED;
	} else {
		retVal.linear.x = -SPEED;
	}

	return retVal;
}

geometry_msgs::Twist turn(char type) {
	geometry_msgs::Twist retVal = stop();

	if (type == 1) {
		retVal.angular.z = SPEED;
	} else {
		retVal.angular.z = -SPEED;
	}

	return retVal;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "command_module");

  NodeHandle handler;
  Subscriber s = handler.subscribe("command_message", 1000, messageHandle);
  Subscriber q = handler.subscribe("odom", 1000, odomReceived);
  Publisher p = handler.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ROS_INFO("** Command Moudule: On-line");

  ROS_INFO("** Command Module: Waiting for Valid Commands");

  Goal goal;
  ros::Rate loop_rate(10);
  while (ros::ok()) {

	if (!GoalFifo.empty()) {
		goal = GoalFifo.front();

		ROS_INFO("** Command Module: Sending Goal");
		if (goal.dist > 1.0) {
			p.publish(move(0));
			ros::Duration(goal.dist).sleep();
			p.publish(stop());
		}

		if (goal.dTheta > 1.0) {
			p.publish(turn(goal.dir));
			ros::Duration(goal.dTheta).sleep();
			p.publish(stop());
		}	


		p.publish(stop());
                GoalFifo.pop();
	}

	ros::spinOnce();
	loop_rate.sleep();
  }
	

  return 0;
}

/**
		if (currentX < goal.X) {
			while (currentX < goal.x) {
                 	  p.publish(move(0));
		   	ROS_INFO("** Command Module CurrentX: %f GoalX:%f", currentX, goal.x);
		   	ros::spinOnce();
                	}
		else {
                	while (currentX > goal.x) {
                 	p.publish(move(1));
		  	ROS_INFO("** Command Module CurrentX: %f GoalX:%f", currentX, goal.x);
		  	ros::spinOnce();
                	}
		}

		if (currentTheta < goal.theta) {
			while (currentTheta < goal.theta) {
				p.publish(turn(goal.dir));
				ros::spinOnce();
				ROS_INFO("** Command Module CurrentTheta: %f GoalTheta:%f", currentTheta, goal.theta);
			}
		} else {
			while (currentTheta > goal.theta) {
				p.publish(turn(goal.dir));
				ros::spinOnce();
				ROS_INFO("** Command Module CurrentTheta: %f GoalTheta:%f", currentTheta, goal.theta);
			}
		} 

**/
