#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "segbot_nlp/VoiceCommand.h"
#include <queue>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef ros::NodeHandle NodeHandle;
typedef ros::Publisher Publisher;
typedef ros::Subscriber Subscriber;

std::queue<move_base_msgs::MoveBaseGoal> GoalFifo;
void messageHandle(const segbot_nlp::VoiceCommand::ConstPtr& msg) {
	/**
	int opcode = msg->commandCode;
	int distance = msg->distance;
	int angle = msg->angle;
	int location = msg->location;
	int numTimes = msg->numTimes;
	**/

	// above are the components of the voice command message
	// we will take these in order to make a new goal for our robot
	move_base_msgs::MoveBaseGoal newGoal; // this is the goal to add to the software fifo here!

	switch(msg->commandCode) {
		default:
			break;
	}

}


int main(int argc, char** argv){
  ros::init(argc, argv, "command_module");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  NodeHandle handler;
 // Subscriber s = handler.subscribe("command_message", 1000, messageHandle);

   ROS_INFO("** Command Moudule: On-line");
  

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward

  ros::Rate loop_rate(10);
  while (1) {

	if (!GoalFifo.empty()) {
		goal = GoalFifo.front();
		goal.target_pose.header.stamp = ros::Time::now();

		ROS_INFO("** Command Module: Sending Goal");
	
		ac.sendGoal(goal);

		ac.waitForResult();

		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("** Command Module: Goal Succeeded");
		} else {
			ROS_INFO("** Command Module: Failure! Shutting Down!");
			return 0;
		}

		GoalFifo.pop();
	}

	ros::spinOnce();
	loop_rate.sleep();
  }
	

  return 0;
}

/*	
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}*/

